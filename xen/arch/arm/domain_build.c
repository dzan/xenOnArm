#include <xen/config.h>
#include <xen/init.h>
#include <xen/compile.h>
#include <xen/lib.h>
#include <xen/mm.h>
#include <xen/domain_page.h>
#include <xen/sched.h>
#include <asm/irq.h>
#include <asm/regs.h>
#include <xen/errno.h>
#include <xen/device_tree.h>
#include <xen/libfdt/libfdt.h>
#include <xen/guest_access.h>
#include <asm/setup.h>

#include <asm/gic.h>
#include "kernel.h"

static unsigned int __initdata opt_dom0_max_vcpus;
integer_param("dom0_max_vcpus", opt_dom0_max_vcpus);

#define DOM0_MEM_DEFAULT (0x8000000*4) /* 128*4 MiB */
static u64 __initdata dom0_mem = DOM0_MEM_DEFAULT;

static void __init parse_dom0_mem(const char *s)
{
    dom0_mem = parse_size_and_unit(s, &s);
    if ( dom0_mem == 0 )
        dom0_mem = DOM0_MEM_DEFAULT;
}
custom_param("dom0_mem", parse_dom0_mem);

/*
 * Amount of extra space required to dom0's device tree.  No new nodes
 * are added (yet) but one terminating reserve map entry (16 bytes) is
 * added.
 */
#define DOM0_FDT_EXTRA_SIZE (128 + sizeof(struct fdt_reserve_entry))

struct vcpu *__init alloc_dom0_vcpu0(void)
{
    if ( opt_dom0_max_vcpus == 0 )
        opt_dom0_max_vcpus = num_online_cpus();
    if ( opt_dom0_max_vcpus > MAX_VIRT_CPUS )
        opt_dom0_max_vcpus = MAX_VIRT_CPUS;

    dom0->vcpu = xzalloc_array(struct vcpu *, opt_dom0_max_vcpus);
    if ( !dom0->vcpu )
        return NULL;
    dom0->max_vcpus = opt_dom0_max_vcpus;

    return alloc_vcpu(dom0, 0, 0);
}

static int set_memory_reg(struct domain *d, struct kernel_info *kinfo,
                          const void *fdt, const u32 *cell, int len,
                          int address_cells, int size_cells, u32 *new_cell)
{
    int reg_size = (address_cells + size_cells) * sizeof(*cell);
    int l = 0;
    u64 start;
    u64 size;

    while ( kinfo->unassigned_mem > 0 && l + reg_size <= len
            && kinfo->mem.nr_banks < NR_MEM_BANKS )
    {
        device_tree_get_reg(&cell, address_cells, size_cells, &start, &size);
        if ( size > kinfo->unassigned_mem )
            size = kinfo->unassigned_mem;
        device_tree_set_reg(&new_cell, address_cells, size_cells, start, size);

        printk("Populate P2M %#"PRIx64"->%#"PRIx64"(1:1 mapping for dom0)\n", start, start + size);
        p2m_populate_ram(d, start, start + size);

        kinfo->mem.bank[kinfo->mem.nr_banks].start = start;
        kinfo->mem.bank[kinfo->mem.nr_banks].size = size;
        kinfo->mem.nr_banks++;
        kinfo->unassigned_mem -= size;

        l += reg_size;
    }

    return l;
}

static int write_properties(struct domain *d, struct kernel_info *kinfo,
                            const void *fdt,
                            int node, const char *name, int depth,
                            u32 address_cells, u32 size_cells)
{
    const char *bootargs = NULL;
    int prop;

    if ( early_info.modules.nr_mods >= 1 &&
         early_info.modules.module[1].cmdline[0] )
        bootargs = &early_info.modules.module[1].cmdline[0];

    for ( prop = fdt_first_property_offset(fdt, node);
          prop >= 0;
          prop = fdt_next_property_offset(fdt, prop) )
    {
        const struct fdt_property *p;
        const char *prop_name;
        const char *prop_data;
        int prop_len;
        char *new_data = NULL;

        p = fdt_get_property_by_offset(fdt, prop, NULL);
        prop_name = fdt_string(fdt, fdt32_to_cpu(p->nameoff));
        prop_data = p->data;
        prop_len  = fdt32_to_cpu(p->len);

        /*
         * In chosen node:
         *
         * * remember xen,dom0-bootargs if we don't already have
         *   bootargs (from module #1, above).
         * * remove bootargs and xen,dom0-bootargs.
         */
        if ( device_tree_node_matches(fdt, node, "chosen") )
        {
            if ( strcmp(prop_name, "bootargs") == 0 )
                continue;
            else if ( strcmp(prop_name, "xen,dom0-bootargs") == 0 )
            {
                if ( !bootargs )
                    bootargs = prop_data;
                continue;
            }
        }
        /*
         * In a memory node: adjust reg property.
         */
        else if ( device_tree_node_matches(fdt, node, "memory") )
        {
            if ( strcmp(prop_name, "reg") == 0 )
            {
                new_data = xzalloc_bytes(prop_len);
                if ( new_data  == NULL )
                    return -FDT_ERR_XEN(ENOMEM);

                prop_len = set_memory_reg(d, kinfo, fdt,
                                          (u32 *)prop_data, prop_len,
                                          address_cells, size_cells,
                                          (u32 *)new_data);
                prop_data = new_data;
            }
        }

        /*
         * TODO: Should call map_mmio_regions() for all devices in the
         * tree that have a "reg" parameter (except cpus).  This
         * requires looking into the parent node's "ranges" property
         * to translate the bus address in the "reg" value into
         * physical addresses.  Regions also need to be rounded up to
         * whole pages.
         */

        fdt_property(kinfo->fdt, prop_name, prop_data, prop_len);

        xfree(new_data);
    }

    if ( device_tree_node_matches(fdt, node, "chosen") && bootargs )
        fdt_property(kinfo->fdt, "bootargs", bootargs, strlen(bootargs) + 1);

    /*
     * XXX should populate /chosen/linux,initrd-{start,end} here if we
     * have module[2]
     */

    if ( prop == -FDT_ERR_NOTFOUND )
        return 0;
    return prop;
}

/* Returns the next node in fdt (starting from offset) which should be
 * passed through to dom0.
 */
static int fdt_next_dom0_node(const void *fdt, int node,
                              int *depth_out)
{
    int depth = *depth_out;

    while ( (node = fdt_next_node(fdt, node, &depth)) &&
            node >= 0 && depth >= 0 )
    {
        if ( depth >= DEVICE_TREE_MAX_DEPTH )
            break;

        /* Skip /hypervisor/ node. We will inject our own. */
        if ( fdt_node_check_compatible(fdt, node, "xen,xen" ) == 0 )
        {
            printk("Device-tree contains \"xen,xen\" node. Ignoring.\n");
            continue;
        }

        /* Skip multiboot subnodes */
        if ( fdt_node_check_compatible(fdt, node,
                                       "xen,multiboot-module" ) == 0 )
            continue;

        /* We've arrived at a node which dom0 is interested in. */
        break;
    }

    *depth_out = depth;
    return node;
}

static void make_hypervisor_node(void *fdt, int addrcells, int sizecells)
{
    const char compat[] =
        "xen,xen-"__stringify(XEN_VERSION)"."__stringify(XEN_SUBVERSION)"\0"
        "xen,xen";
    u32 reg[4];
    u32 intr[3];
    u32 *cell;

    /*
     * Sanity-check address sizes, since addresses and sizes which do
     * not take up exactly 4 or 8 bytes are not supported.
     */
    if ((addrcells != 1 && addrcells != 2) ||
        (sizecells != 1 && sizecells != 2))
        panic("Cannot cope with this size");

    /* See linux Documentation/devicetree/bindings/arm/xen.txt */
    fdt_begin_node(fdt, "hypervisor");

    /* Cannot use fdt_property_string due to embedded nulls */
    fdt_property(fdt, "compatible", compat, sizeof(compat) + 1);

    /* reg 0 is grant table space */
    cell = &reg[0];
    device_tree_set_reg(&cell, addrcells, sizecells, 0xb0000000, 0x20000);
    fdt_property(fdt, "reg", reg,
                 sizeof(reg[0]) * (addrcells + sizecells));

    /*
     * interrupts is evtchn upcall  <1 15 0xf08>
     * See linux Documentation/devicetree/bindings/arm/gic.txt
     */
    intr[0] = cpu_to_fdt32(1); /* is a PPI */
    intr[1] = cpu_to_fdt32(VGIC_IRQ_EVTCHN_CALLBACK - 16); /* PPIs start at 16 */
    intr[2] = cpu_to_fdt32(0xf08); /* Active-low level-sensitive */

    fdt_property(fdt, "interrupts", intr, sizeof(intr[0]) * 3);

    fdt_end_node(fdt);
}

static int write_nodes(struct domain *d, struct kernel_info *kinfo,
                       const void *fdt)
{
    int node;
    int depth = 0, last_depth = -1;
    u32 address_cells[DEVICE_TREE_MAX_DEPTH];
    u32 size_cells[DEVICE_TREE_MAX_DEPTH];
    int ret;

    for ( node = 0, depth = 0;
          node >= 0 && depth >= 0;
          node = fdt_next_dom0_node(fdt, node, &depth) )
    {
        const char *name;

        name = fdt_get_name(fdt, node, NULL);

        if ( depth >= DEVICE_TREE_MAX_DEPTH )
        {
            printk("warning: node `%s' is nested too deep (%d)\n",
                   name, depth);
            continue;
        }

        /* We cannot handle descending more than one level at a time */
        ASSERT( depth <= last_depth + 1 );

        while ( last_depth-- >= depth )
            fdt_end_node(kinfo->fdt);

        address_cells[depth] = device_tree_get_u32(fdt, node, "#address-cells",
                                    depth > 0 ? address_cells[depth-1] : 0);
        size_cells[depth] = device_tree_get_u32(fdt, node, "#size-cells",
                                    depth > 0 ? size_cells[depth-1] : 0);

        fdt_begin_node(kinfo->fdt, name);

        ret = write_properties(d, kinfo, fdt, node, name, depth,
                               address_cells[depth-1], size_cells[depth-1]);
        if ( ret < 0 )
            return ret;

        last_depth = depth;
    }

    while ( last_depth-- >= 1 )
        fdt_end_node(kinfo->fdt);

    make_hypervisor_node(kinfo->fdt, address_cells[0], size_cells[0]);

    fdt_end_node(kinfo->fdt);
    return 0;
}

static int prepare_dtb(struct domain *d, struct kernel_info *kinfo)
{
    void *fdt;
    int new_size;
    int ret;

    kinfo->unassigned_mem = dom0_mem;

    fdt = device_tree_flattened;

    new_size = fdt_totalsize(fdt) + DOM0_FDT_EXTRA_SIZE;
    kinfo->fdt = xmalloc_bytes(new_size);
    if ( kinfo->fdt == NULL )
        return -ENOMEM;

    ret = fdt_create(kinfo->fdt, new_size);
    if ( ret < 0 )
        goto err;

    fdt_finish_reservemap(kinfo->fdt);

    ret = write_nodes(d, kinfo, fdt);
    if ( ret < 0 )
        goto err;

    ret = fdt_finish(kinfo->fdt);
    if ( ret < 0 )
        goto err;

    /*
     * Put the device tree at the beginning of the first bank.  It
     * must be below 4 GiB.
     */
    kinfo->dtb_paddr = kinfo->mem.bank[0].start + 0x100;
    if ( kinfo->dtb_paddr + fdt_totalsize(kinfo->fdt) > (1ull << 32) )
    {
        printk("Not enough memory below 4 GiB for the device tree.");
        ret = -FDT_ERR_XEN(EINVAL);
        goto err;
    }

    return 0;

  err:
    printk("Device tree generation failed (%d).\n", ret);
    xfree(kinfo->fdt);
    return -EINVAL;
}

static void dtb_load(struct kernel_info *kinfo)
{
    void * __user dtb_virt = (void * __user)(register_t)kinfo->dtb_paddr;

    raw_copy_to_guest(dtb_virt, kinfo->fdt, fdt_totalsize(kinfo->fdt));
    xfree(kinfo->fdt);
}

int construct_dom0(struct domain *d)
{
    struct kernel_info kinfo = {};
    int rc, i, cpu = 0;

    struct vcpu *v = d->vcpu[0];
    struct cpu_user_regs *regs = &v->arch.cpu_info->guest_cpu_user_regs;

    /* Sanity! */
    BUG_ON(d->domain_id != 0);
    BUG_ON(d->vcpu[0] == NULL);
    BUG_ON(v->is_initialised);

    printk("*** LOADING DOMAIN 0 ***\n");

    d->max_pages = ~0U;

#ifdef EARLY_RAMOOPS_ADDRESS
    // must save kernel out of dom0 space
    {
        paddr_t s = early_info.modules.module[1].start;
        paddr_t size = early_info.modules.module[1].size;
        paddr_t e = (s + early_info.modules.module[1].size)&PAGE_MASK;
        kinfo.kernel_img = xmalloc_bytes(size+1);
        copy_from_paddr(kinfo.kernel_img, s, size, DEV_SHARED);
        printk("DEBUG %s %d, free at %#010llx-%#010llx\n", __func__, __LINE__, s&PAGE_MASK, e);
        init_domheap_pages(s&PAGE_MASK, e);
    }
    printk("DEBUG %s %d\n", __func__, __LINE__);
#endif

    rc = prepare_dtb(d, &kinfo);
    if ( rc < 0 )
        return rc;

    // for the debug trap vector
    p2m_populate_ram(d, 0x0, 0x1000-1);
    rc = kernel_prepare(&kinfo);
    if ( rc < 0 )
        return rc;

#ifdef EARLY_RAMOOPS_ADDRESS
    map_mmio_regions(d, 0x0b0000, 0x0b0000+0x020000-1, 0x0b0000);
    map_mmio_regions(d, 0x03800000, 0x03870000-1, 0x03800000);
#endif
    printk("Map CS2 MMIO regions 1:1 in the P2M %#llx->%#llx\n", 0x18000000ULL, 0x1BFFFFFFULL);
    map_mmio_regions(d, 0x18000000, 0x1BFFFFFF, 0x18000000);
    printk("Map CS3 MMIO regions 1:1 in the P2M %#llx->%#llx\n", 0x1C000000ULL, 0x1FFFFFFFULL);
    map_mmio_regions(d, 0x1C000000, 0x1FFFFFFF, 0x1C000000);

    // iRam
    map_mmio_regions(d, 0x02020000, 0x02077fff, 0x02020000);
    printk("map chip id to 0x10000000\n");
    map_mmio_regions(d, 0x10000000, 0x10010000, 0x10000000);
    printk("map more\n");
    map_mmio_regions(d, 0x10010000, 0x101c0000-1, 0x10010000);
#ifdef EARLY_RAMOOPS_ADDRESS
    map_mmio_regions(d, 0x101c0000, 0x101d0000-1, 0x101c0000);//MCT
#endif
    map_mmio_regions(d, 0x101d0000, 0x10480000-1, 0x101d0000);
    // avoid GIC
    map_mmio_regions(d, 0x10490000, 0x121fffff, 0x10490000);
    // MSH[0-3]:
    map_mmio_regions(d, 0x12200000, 0x1223ffff, 0x12200000);
    map_mmio_regions(d, 0x12240000, 0x12dd0000-1, 0x12240000);
    map_mmio_regions(d, 0x12dd0000, 0x12de0000-1, 0x12dd0000); // pwm
    map_mmio_regions(d, 0x12de0000, 0x12efffff, 0x12de0000);
#ifdef EARLY_RAMOOPS_ADDRESS
    map_mmio_regions(d, 0x12c00000, 0x12c0ffff, 0x12c00000); // uart0
    map_mmio_regions(d, 0x12c10000, 0x12c1ffff, 0x12c10000); // uart1
    // avoid uart2, used by Xen
    map_mmio_regions(d, 0x12c20000, 0x12C2ffff, 0x12c20000); // uart2
    map_mmio_regions(d, 0x12c30000, 0x12C3ffff, 0x12c30000); // uart3
#endif
    map_mmio_regions(d, 0x12c40000, 0x17ffffff, 0x12c40000);

    printk("Routing peripheral interrupts to guest\n");
    /* TODO Get from device tree */


    // Combined Interrupt
    for ( i = 0; i < 32; i++ )
        gic_route_irq_to_guest(d, 32+i, "intc");
    gic_route_irq_to_guest(d, 64, "eint16_31");
    gic_route_irq_to_guest(d, 65, "mdma0_core");
    gic_route_irq_to_guest(d, 66, "pdma0");
    gic_route_irq_to_guest(d, 67, "pdma1");

    gic_route_irq_to_guest(d, 68, "timer0");
    gic_route_irq_to_guest(d, 69, "timer1");
    gic_route_irq_to_guest(d, 70, "timer2");
    gic_route_irq_to_guest(d, 71, "timer3");
    gic_route_irq_to_guest(d, 72, "timer4");
    gic_route_irq_to_guest(d, 73, "rtic");
    gic_route_irq_to_guest(d, 74, "wdt");
    gic_route_irq_to_guest(d, 75, "rtc_alarm");
    gic_route_irq_to_guest(d, 76, "rtc_tic");
    gic_route_irq_to_guest(d, 77, "gpio_rt");
    gic_route_irq_to_guest(d, 78, "gpio_bl");
    gic_route_irq_to_guest(d, 79, "gpio");
    gic_route_irq_to_guest(d, 82, "gpio_c2c");

    gic_route_irq_to_guest(d, 83, "uart0");
    gic_route_irq_to_guest(d, 84, "uart1");
    //gic_route_irq_to_guest(d, 85, "uart2"); /* -- XXX used by Xen*/
    gic_route_irq_to_guest(d, 86, "uart3");

    gic_route_irq_to_guest(d, 87, "monocnt");

    gic_route_irq_to_guest(d, 88, "i2c0");
    gic_route_irq_to_guest(d, 89, "i2c1");
    gic_route_irq_to_guest(d, 90, "i2c2");
    gic_route_irq_to_guest(d, 91, "i2c3");
    gic_route_irq_to_guest(d, 92, "i2c4");
    gic_route_irq_to_guest(d, 93, "i2c5");
    gic_route_irq_to_guest(d, 94, "i2c6");
    gic_route_irq_to_guest(d, 95, "i2c7");
    gic_route_irq_to_guest(d, 96, "i2c hdmi");

    gic_route_irq_to_guest(d, 97, "tmu");

    gic_route_irq_to_guest(d, 98, "cpu_nfiq0");
    gic_route_irq_to_guest(d, 99, "cpu_nfiq1");

    gic_route_irq_to_guest(d, 100, "spi0");
    gic_route_irq_to_guest(d, 101, "spi1");
    gic_route_irq_to_guest(d, 102, "spi2");

    gic_route_irq_to_guest(d, 103, "usb-host");
    gic_route_irq_to_guest(d, 104, "usb-drd");

    gic_route_irq_to_guest(d, 105, "mipi_hsi");
    gic_route_irq_to_guest(d, 106, "usbotg");

    gic_route_irq_to_guest(d, 107, "sdmmc0");
    gic_route_irq_to_guest(d, 108, "sdmmc1");
    gic_route_irq_to_guest(d, 109, "sdmmc2");
    gic_route_irq_to_guest(d, 110, "sdmmc3");

    gic_route_irq_to_guest(d, 111, "mipi_csi_a");
    gic_route_irq_to_guest(d, 112, "mipi_csi_b");

    gic_route_irq_to_guest(d, 113, "efnfcon_dma_abort");
    gic_route_irq_to_guest(d, 114, "mipi_dsi_4lane");
    gic_route_irq_to_guest(d, 115, "wdt_iop");
    gic_route_irq_to_guest(d, 116, "rotator");

    for ( i = 0; i <= 3; i++ )
        gic_route_irq_to_guest(d, 117+i, "gscl");

    gic_route_irq_to_guest(d, 121, "jpeg");
    gic_route_irq_to_guest(d, 122, "efnfcon_dma");
    gic_route_irq_to_guest(d, 123, "g2d");
    gic_route_irq_to_guest(d, 124, "efnfcon_0");
    gic_route_irq_to_guest(d, 125, "efnfcon_1");
    gic_route_irq_to_guest(d, 126, "mixer");
    gic_route_irq_to_guest(d, 127, "hdmi");
    gic_route_irq_to_guest(d, 128, "mfc");
    // audio_ss
    for ( i = 0; i <= 2; i++ )
        gic_route_irq_to_guest(d, 130+i, "i2s");

    // 133 ac97
    for ( i = 0; i <= 2; i++ )
        gic_route_irq_to_guest(d, 134+i, "pcm");
    // spdif
    gic_route_irq_to_guest(d, 138, "adc0");
    // reserved
    gic_route_irq_to_guest(d, 140, "sataphy");
    gic_route_irq_to_guest(d, 141, "satapmereq");
    gic_route_irq_to_guest(d, 142, "cam_c");
    gic_route_irq_to_guest(d, 143, "pmu");
    gic_route_irq_to_guest(d, 144, "intfeedctrl_sss");
    gic_route_irq_to_guest(d, 145, "dp1_1");
    gic_route_irq_to_guest(d, 146, "cec");
    gic_route_irq_to_guest(d, 147, "sata");
    // reserved
    gic_route_irq_to_guest(d, 149, "g3d_irqgpu");
    gic_route_irq_to_guest(d, 150, "g3d_irqjob");
    gic_route_irq_to_guest(d, 151, "g3d_irqmmu");

#ifdef EARLY_RAMOOPS_ADDRESS
    gic_route_irq_to_guest(d, 152, "mct-l0");
    gic_route_irq_to_guest(d, 153, "mct-l1");
#endif
    // reserved
    gic_route_irq_to_guest(d, 156, "mdma1");

    gic_route_irq_to_guest(d, 157, "cam_a");
    gic_route_irq_to_guest(d, 158, "cam_b");

    gic_route_irq_to_guest(d, 159, "rp_timer");

    /* The following loads use the domain's p2m */
    p2m_load_VTTBR(d);

    kinfo.dtb_paddr = kinfo.zimage.load_addr + kinfo.zimage.len;
    kernel_load(&kinfo);
    dtb_load(&kinfo);

#ifdef EARLY_RAMOOPS_ADDRESS
    xfree(kinfo.kernel_img);
#endif

    //discard_initial_modules();

    clear_bit(_VPF_down, &v->pause_flags);

    memset(regs, 0, sizeof(*regs));

    regs->pc = (uint32_t)kinfo.entry;

    regs->cpsr = PSR_ABT_MASK|PSR_FIQ_MASK|PSR_IRQ_MASK|PSR_MODE_SVC;

#ifdef CONFIG_ARM_64
    d->arch.type = kinfo.type;
#endif

    if ( is_pv32_domain(d) )
    {
        /* FROM LINUX head.S
         *
         * Kernel startup entry point.
         * ---------------------------
         *
         * This is normally called from the decompressor code.  The requirements
         * are: MMU = off, D-cache = off, I-cache = dont care, r0 = 0,
         * r1 = machine nr, r2 = atags or dtb pointer.
         *...
         */
        regs->r0 = 0; /* SBZ */
        regs->r1 = 0xffffffff; /* We use DTB therefore no machine id */
        regs->r2 = kinfo.dtb_paddr;
    }
#ifdef CONFIG_ARM_64
    else
    {
        /* From linux/Documentation/arm64/booting.txt */
        regs->x0 = kinfo.dtb_paddr;
        regs->x1 = 0; /* Reserved for future use */
        regs->x2 = 0; /* Reserved for future use */
        regs->x3 = 0; /* Reserved for future use */
    }
#endif

    for ( i = 1; i < d->max_vcpus; i++ )
    {
        cpu = cpumask_cycle(cpu, &cpu_online_map);
        (void)alloc_vcpu(d, i, cpu);
    }

    local_abort_enable();

    return 0;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "BSD"
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */

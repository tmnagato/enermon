#include <linux/init.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
MODULE_LICENSE("Dual BSD/GPL");


#define GPIO_REG_BASE   0xFF008000
#define GPIO_MEM_LEN    192
#define REG_GPLR_OFFSET 0x04

enum GPIO_REG
{
	GPLR = 0,   // pin level read-only
	GPDR,       // pin direction
    GPSR,       // pin set
    GPCR,       // pin clear
    GRER,       //rising edge detect
    GFER,       //falling edge detect
    GEDR,       // edge detect result
    GAFR        //alt function
} ;

#define DRIVER_NAME	"edison_gpio"

#define NGPIO 192

void __iomem *reg_base;
void __iomem *gpio;


static void __iomem *gpio_reg(unsigned offset, enum GPIO_REG reg_type)
{
    unsigned nreg = NGPIO / 32;
    u8 reg = offset / 32;
    
    return reg_base + reg_type * nreg * 4 + reg * 4; 
}

static void __iomem *gpio_reg_2bit(unsigned offset, enum GPIO_REG reg_type)
{
    unsigned nreg = NGPIO / 32;
    u8 reg = offset / 16;
    
    return reg_base + reg_type * nreg * 4 + reg * 4; 
}

static int gpio_req(unsigned offset)
{
    void __iomem *gafr = gpio_reg_2bit(offset, GAFR);
    u32 value = readl(gafr);
    int shift = (offset %16 )<<1, af = (value >> shift) & 3;
    if(af)
    {
        value &= ~ (3 << shift);
        writel(value, gafr);
    }
    
    return 0; 
}

static int hello_init(void)
{       
    printk(KERN_ALERT "Hello, world\n");
    
    //map io memory
    gpio = ioremap_nocache(GPIO_REG_BASE, GPIO_MEM_LEN);
    
    if(gpio == NULL)
    {
        printk(KERN_ALERT"We were no able to map GPIO region\n");
        return -1;
    }
    
    reg_base = gpio + REG_GPLR_OFFSET;
 
    
    printk(KERN_ALERT"GPIO_REG_BASE: 0x%x\n", GPIO_REG_BASE);
    printk(KERN_ALERT"reg_base: 0x%x\n", reg_base);
    
    unsigned offset = 44;
    
    void __iomem *gplr = gpio_reg( offset, GPLR );
    void __iomem *gpdr = gpio_reg( offset, GPDR );
    void __iomem *gpsr = gpio_reg( offset, GPSR );
    void __iomem *gpcr = gpio_reg( offset, GPCR );
    
    printk(KERN_ALERT"gplr:0x%x: 0x%x\n",gplr, readl(gplr) & BIT(offset%32));
    printk(KERN_ALERT"gpdr:0x%x: 0x%x\n",gpdr, readl(gpdr) & (1<<(offset%32)));
    printk(KERN_ALERT"gpsr:0x%x: 0x%x\n",gpsr, readl(gpsr) & (1<<(offset%32)));
    printk(KERN_ALERT"gpcr:0x%x: 0x%x\n",gpcr, readl(gpcr) & (1<<(offset%32)));
    
    //set as as GPIO
//    gpio_req(offset);
    
    //set direction as output
 //   writel( readl( gpdr ) | (1<<(offset % 32)), gpdr );
    
    //set to one
 //   writel( 1 << (offset % 32), gpcr);
 
	
    return 0;
}

static void hello_exit(void)
{
    iounmap(gpio);
    printk(KERN_ALERT  "Goodbye, cruel world\n");
}
module_init(hello_init);
module_exit(hello_exit);



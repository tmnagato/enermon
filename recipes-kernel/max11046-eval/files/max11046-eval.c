#include <linux/init.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
MODULE_LICENSE("Dual BSD/GPL");


//ATTENTION:
//pwm linux device driver does not set the corresponding alternative function
//in the GPIO, so, we have 2 options:
//1) every time we do: echo mode1 > /sys/kernel/debug/gpio_debug/gpio183/current_pinmux
//2) setup mode1 in this driver by hand (GAFR register does not work, we need to go through FLIS registers. See langwell gpio driver).

// Edison GPIO base address. This address was collected from PCI device.
#define GPIO_REG_BASE   0xFF008000
//Edison number of GPIO
#define NGPIO 192
//Memory size for gpio registers
#define GPIO_MEM_LEN    ((NGPIO/32)*9*4 + 4)    // we have 9 GPIO reg for each GPIO block of 32.; +4 because we start at the REG_GPLR_OFFSET
#define REG_GPLR_OFFSET 0x04

//where each data bus bit is connected to
#define GP_DB_15    47
#define GP_DB_14    48
#define GP_DB_13    49
#define GP_DB_12    15
#define GP_DB_11    84
#define GP_DB_10    14
#define GP_DB_09    42
#define GP_DB_08    40
#define GP_DB_07    41
#define GP_DB_06    43
#define GP_DB_05    78
#define GP_DB_04    77
#define GP_DB_03    79
#define GP_DB_02    80
#define GP_DB_01    82
#define GP_DB_00    81

//where each control signal is connected to
#define GP_WR       44
#define GP_CS       45
#define GP_RD       46
#define GP_EOC      83
#define GP_CONVST   183
#define GP_SHDN     114

#define PWM_NR  3

struct DataBit
{
    u8 reg;    //register index
    u8 bit;      //bit in the register
    u8 db;       //data bit (of data bus)
    u8 shift;    //shift amount (bit - db)
};

#define DATABUS_LEN  16
struct DataBit data_bit[DATABUS_LEN] = 
{
    //  [i] = .reg = GP_DB_i / 32, .bit = GP_DB_i % 32, .db = i, .shift = .bit - .db
    [15] = {.reg = 1, .bit = 15, .db = 15, .shift = 0},
    [14] = {.reg = 1, .bit = 16, .db = 14, .shift = 2},
    [13] = {.reg = 1, .bit = 17, .db = 13, .shift = 4},
    [12] = {.reg = 0, .bit = 15, .db = 12, .shift = 3},
    [11] = {.reg = 2, .bit = 20, .db = 11, .shift = 9},
    [10] = {.reg = 0, .bit = 14, .db = 10, .shift = 4},
    [9] = {.reg = 1, .bit = 10, .db = 9, .shift = 1},
    [8] = {.reg = 1, .bit = 8, .db = 8, .shift = 0},
    [7] = {.reg = 1, .bit = 9, .db = 7, .shift = 2},
    [6] = {.reg = 1, .bit = 11, .db = 6, .shift = 5},
    [5] = {.reg = 2, .bit = 14, .db = 5, .shift = 9},
    [4] = {.reg = 2, .bit = 13, .db = 4, .shift = 9},
    [3] = {.reg = 2, .bit = 15, .db = 3, .shift = 12},
    [2] = {.reg = 2, .bit = 16, .db = 2, .shift = 14},
    [1] = {.reg = 2, .bit = 18, .db = 1, .shift = 17},
    [0] = {.reg = 2, .bit = 17, .db = 0, .shift = 17},
};

#define GP_R0_IDX   0
#define GP_R1_IDX   32
#define GP_R2_IDX   64
#define GP_R3_IDX   96
#define GP_R4_IDX   128
#define GP_R5_IDX   160

// General purpose Register 0 (first bank of GPIO) Data Bus Mask
#define GP_R0_DB_MASK ( (1 << (GP_DB_10 % 32)) | (1 << (GP_DB_12 % 32) ))

// General purpose Register 1 (second bank of GPIO) Data Bus Mask
#define GP_R1_DB_MASK ( (1 << (GP_DB_08 % 32)) | (1 << (GP_DB_07 % 32)) | (1 << (GP_DB_09 % 32)) \
                      | (1 << (GP_DB_06 % 32)) | (1 << (GP_DB_15 % 32)) | (1 << (GP_DB_14 % 32)) \
                      | (1 << (GP_DB_13 % 32)) )

// General purpose Register 2 (third bank of GPIO) Data Bus Mask
#define GP_R2_DB_MASK ( (1 << (GP_DB_04 % 32)) | (1 << (GP_DB_05 % 32)) | (1 << (GP_DB_03 % 32)) \
                      | (1 << (GP_DB_02 % 32)) | (1 << (GP_DB_00 % 32)) | (1 << (GP_DB_01 % 32)) \
                      | (1 << (GP_DB_11 % 32)) )
                      
// General purpose Register 2 Data Bus Mask for configuration of MAX11046                      
#define GP_R2_DB_CF_MASK ( (1 << (GP_DB_00 % 32)) | (1 << (GP_DB_01 % 32)) \
                         | (1 << (GP_DB_02 % 32)) | (1 << (GP_DB_03 % 32)) )
                  
//Tangier chip GPIO registers
enum GPIO_REG
{
	GPLR = 0,   // pin level read-only
	GPDR,       // pin direction
    GPSR,       // pin set
    GPCR,       // pin clear
    GRER,       // rising edge detect
    GFER,       // falling edge detect
    GEDR,       // edge detect result
    GAFR        // alt function
} ;

#define DRIVER_NAME	"max11046"

//store gpio controller base address
void __iomem *gpio;
//store gpio registers base address
void __iomem *reg_base;
//store pwm pointer
struct pwm_device *pwm;
//store eoc irq line
int eoc_irq;


// text below will be seen in 'cat /proc/interrupt' command
#define GPIO_INTERRUPT_DESC           "external interrupt for the max11046 device (signals EOC)"


//Get the address of the reg_type register to the specific offset gpio
static inline void __iomem *_gpio_reg(unsigned offset, enum GPIO_REG reg_type)
{
    unsigned nreg = NGPIO / 32;
    u8 reg = offset / 32;
    
    return reg_base + reg_type * nreg * 4 + reg * 4; 
}

//Get the address of the reg_type register to the specific offset gpio (each offset ocupies 2 bit), used e.g. for alternative gp function
static inline void __iomem *_gpio_reg_2bit(unsigned offset, enum GPIO_REG reg_type)
{
    unsigned nreg = NGPIO / 32;
    u8 reg = offset / 16;
    
    return reg_base + reg_type * nreg * 4 + reg * 4; 
}

//Set the specific gpio offset to the GPIO alternative function
/*static int _gpio_req(unsigned offset)
{
    void __iomem *gafr = _gpio_reg_2bit(offset, GAFR);
    u32 value = readl(gafr);
    int shift = (offset %16 )<<1, af = (value >> shift) & 3;
    if(af)
    {
        value &= ~ (3 << shift);
        writel(value, gafr);
    }
    
    return 0; 
}*/

static inline void _gpio_direction_input(unsigned offset)
{
    u32 value;
    void __iomem *gpdr = _gpio_reg( offset, GPDR);
    
    value = ioread32( gpdr );
    value &= ~ BIT(offset % 32);
    iowrite32( value, gpdr );
}

static inline void _gpio_set(unsigned offset, int value)
{
    void __iomem *gpcr, *gpsr;
    
    if(value)
    {
        gpsr = _gpio_reg( offset, GPSR);
        iowrite32( BIT(offset%32), gpsr);
    }
    else
    {
        gpcr = _gpio_reg( offset, GPCR);
        iowrite32( BIT(offset%32), gpcr);  
    }
}

static inline void _gpio_set_low(unsigned offset)
{
    void __iomem *gpcr;
    gpcr = _gpio_reg( offset, GPCR);
    iowrite32( BIT(offset%32), gpcr);
}

static inline void _gpio_set_high(unsigned offset)
{
    void __iomem *gpsr;
    gpsr = _gpio_reg( offset, GPSR);
    iowrite32( BIT(offset%32), gpsr);
}

static inline void _gpio_direction_output(unsigned offset, int value)
{
    void __iomem *gpdr = _gpio_reg( offset, GPDR);
    
    _gpio_set(offset, value);
    
    value = ioread32( gpdr );
    value |= BIT(offset % 32);
    iowrite32( value, gpdr );
}

/*IRQ Handler*/
static irqreturn_t read_stream(int irq, void *dev_id)
{
    static int count = 0;
    unsigned i,b;
    unsigned val = 0;
    unsigned vals[8] = {0};
    unsigned data_array[3];
    void __iomem *gplr0 = _gpio_reg(GP_R0_IDX, GPLR);
    void __iomem *gplr1 = _gpio_reg(GP_R1_IDX, GPLR);
    void __iomem *gplr2 = _gpio_reg(GP_R2_IDX, GPLR);
   
    if( count ++ == 10)
    {
       
        printk(KERN_ALERT "Hello, world from interrupt\n");

        //CS = 0
        _gpio_set_low(GP_CS);

        for(i = 0 ;i < 8; i++)
        {
            //RD = 0
            _gpio_set_low(GP_RD);
            udelay(1);
            
            //ioread32_rep(gplr0, data_array, 3);
            
            data_array[0] = ioread32( gplr0 );
            data_array[1] = ioread32( gplr1 );
            data_array[2] = ioread32( gplr2 );
            val = 0;
            //reorder data bits
            for(b = 0; b < DATABUS_LEN; b++)
                val |= (data_array[data_bit[b].reg] & data_bit[b].bit ) >> data_bit[b].shift;

            //sample = (u16)(Dados_mask & *( (unsigned *)(PIOB+PDSR) ) );
            //sample = (u16)(Dados_mask & __raw_readl(PIOB + PDSR));

            // samples[i] = (u8)(sample >> 8);
            // samples[i + 1] = (u8)sample;
            
            //  printk(KERN_ALERT "val: 0x%08x\n", val);
            // printk(KERN_ALERT "val: 0x%08x\n", val);
            vals[i] = val;
            
            //RD = 1
            _gpio_set_high(GP_RD);
            udelay(1);
        }

        //CS = 1
        _gpio_set_high(GP_CS);

        for(i=0; i<8; i++)
            printk(KERN_ALERT "val: 0x%04x\n", vals[i]);
        
        count = 0;
    }
    return IRQ_HANDLED;
}


static int max11046_init(void)
{       
    void __iomem *gpdr, *gpcr, *gpsr;
    u32 val;
    int err;
    
    printk(KERN_ALERT "Hello, world\n");
    
    //map GPIO IO memory
    gpio = ioremap_nocache(GPIO_REG_BASE, GPIO_MEM_LEN);
    
    if(gpio == NULL)
    {
        printk(KERN_ALERT"We were no able to map GPIO region\n");
        return -1;
    }
    
    reg_base = gpio + REG_GPLR_OFFSET;
    
    //disable pwm
    pwm = pwm_request(PWM_NR, DRIVER_NAME);
    
    if(IS_ERR(pwm))
    {
        printk(KERN_ALERT"Failed to get pwm\n");
        iounmap(gpio);
        return -1;
    }
    
    pwm_disable(pwm); 
            
    err =  pwm_config( pwm, 218453000/2, 218453000);
    if(err)
    {
        printk(KERN_ALERT "error %d: failed to configure\n", err);
        pwm_free(pwm);
        iounmap(gpio);
        return -1;
    }
    
    //install an irq handler for pin EOC
    eoc_irq = gpio_to_irq( GP_EOC );

    if (request_irq(eoc_irq, read_stream,  IRQF_TRIGGER_FALLING, GPIO_INTERRUPT_DESC, NULL)) //inicializar o IRQ
    {
        printk(KERN_ALERT "error %d: could not request irq: %d\n", -EBUSY, GP_EOC);
        pwm_free(pwm);
        iounmap(gpio);
        return -EBUSY;
    }    
    
    // Configure interface with MAX11046

    printk(KERN_ALERT "inicializando o max\n");

    //Configure data bus as input
    gpdr = _gpio_reg( GP_R0_IDX, GPDR );
    val = ioread32(gpdr);
    val &= ~ GP_R0_DB_MASK;
    iowrite32 ( val, gpdr);
    
    gpdr = _gpio_reg( GP_R1_IDX, GPDR );
    val = ioread32(gpdr);
    val &= ~ GP_R1_DB_MASK;
    iowrite32 ( val, gpdr);
    
    gpdr = _gpio_reg( GP_R2_IDX, GPDR );
    val = ioread32(gpdr);
    val &= ~ GP_R2_DB_MASK;
    iowrite32 ( val, gpdr);


    //configure SHDN, CONVST, RD, CS and WR as output
    //before, ensure all bits are high so that MAX does not accept anything, these are the values writen to pin as soon as we enable output

    //shutdown MAX, shdn <- 1
    _gpio_direction_output( GP_SHDN, 1);

    // cs <- 1
    _gpio_direction_output( GP_CS, 1);

    //disable write, wr <- 1
    _gpio_direction_output( GP_WR, 1);

    //disable read, rd <- 1
    _gpio_direction_output( GP_RD, 1);




    //enable pullup
 //   iowrite32( 1, gpio + GPPUD);
//    udelay(150);

 //   iowrite32( MAX_DATA_BUS, gpio + GPPUDCLK0);
 //   udelay(150);

  //  iowrite32( 0, gpio + GPPUD);
  //   iowrite32( 0, gpio + GPPUDCLK0);

    //===  configure MAX  ===

    //set DB0-3 as out
    gpdr = _gpio_reg( GP_R2_IDX, GPDR );
    val = ioread32(gpdr);
    val |= GP_R2_DB_CF_MASK;
    iowrite32 ( val, gpdr);

    //write:
    // DB0 = '0' (CONVST NA AQUISIÇÃO
    // DB1='0' (RESERVADO)
    // DB2= '0' (OFFSET BINÁRIO)
    // DB3 = '0' REFERENCIA INTERNA

    gpcr = _gpio_reg( GP_R2_IDX, GPCR);
    val = GP_R2_DB_CF_MASK;
    iowrite32( val, gpcr );

    //bring max up, shdn <- 0
    _gpio_set_low( GP_SHDN);

    //select the device, cs <- 0
    _gpio_set_low(GP_CS);

    //enable write, wr <- 0
    _gpio_set_low(GP_WR);
    udelay(1);

    //disable write, wr <- 1
    _gpio_set_high(GP_WR);


    //disable chip cs <- 1
    _gpio_set_high(GP_CS);


    //=== all data bus as input ===
    //set DB0-3 as in
    gpdr = _gpio_reg( GP_R2_IDX, GPDR );
    val = ioread32(gpdr);
    val &= ~GP_R2_DB_CF_MASK;
    iowrite32 ( val, gpdr);

    // start PWM3
    err = pwm_enable(pwm); 
    
    if(err)
    {
        printk(KERN_ALERT "error %d: failed to enable\n", err);
        free_irq(eoc_irq, NULL);
        pwm_free(pwm);
        iounmap(gpio);
        return -1;
    }

    return 0;
}

static void max11046_exit(void)
{
    free_irq(eoc_irq, NULL);
    pwm_disable(pwm);
    pwm_free(pwm);
    
    iounmap(gpio);
    printk(KERN_ALERT  "Goodbye, cruel world\n");
}
module_init(max11046_init);
module_exit(max11046_exit);



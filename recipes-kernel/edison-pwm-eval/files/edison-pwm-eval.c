#include <linux/init.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
MODULE_LICENSE("Dual BSD/GPL");

//ATTENTION:
//pwm linux driver driver does not set the corresponding alternative function
//in the GPIO, so, we have 2 options:
//1) every time we do: echo mode1 > /sys/kernel/debug/gpio_debug/gpio183/current_pinmux
//2) setup mode1 in this driver by hand.

#define DRIVER_NAME	"edison-pwm-eval"

// we want GPIO_17
// to generate interrupt
#define GPIO_INTERRUPT                44

// text below will be seen in 'cat /proc/interrupt' command
#define GPIO_INTERRUPT_DESC           "external interrupt connected to pwm (do the shunt!)"

struct pwm_device *pwm;

int gpio_int_irq = 0;

/*IRQ Handler*/
static irqreturn_t read_stream(int irq, void *dev_id)
{
    static int count = 0;
   
    if( count ++ == 10)
    {
        printk(KERN_ALERT "Hello, world from interrupt\n");
        count = 0;
    }
    return IRQ_HANDLED;
}


static int eval_init(void)
{       
    int err;
    printk(KERN_ALERT "Hello, world\n");
    
    //install an irq handler for pin 44
    gpio_int_irq = gpio_to_irq( GPIO_INTERRUPT );

    if (request_irq(gpio_int_irq, read_stream,  IRQF_TRIGGER_FALLING, GPIO_INTERRUPT_DESC, NULL)) //inicializar o IRQ
    {
        printk(KERN_ALERT "error %d: could not request irq: %d\n", -EBUSY,GPIO_INTERRUPT);
        return -EBUSY;
    }
    
    pwm = pwm_request(3, DRIVER_NAME);
    
    if(IS_ERR(pwm))
    {
         printk(KERN_ALERT"Failed to get pwm\n");
        return -1;
    }
    err =  pwm_config( pwm, 218453000/2, 218453000);
    if(err)
     printk(KERN_ALERT "error %d: failed to configure\n", err);
    err = pwm_enable(pwm); 
    
    if(err)
     printk(KERN_ALERT "error %d: failed to enable\n", err);
	
    return 0;
}

static void eval_exit(void)
{
    
    free_irq(gpio_int_irq, NULL);

    pwm_disable(pwm);
    pwm_free(pwm);
    printk(KERN_ALERT  "Goodbye, cruel world\n");
}
module_init(eval_init);
module_exit(eval_exit);



#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>

#define HALL_DRIVER_NAME "sensor_hall"
#define HALL_GPIO_INT 140
static struct input_dev *hall_input;
static void hall_check_func(struct work_struct *work);
static DECLARE_WORK(hall_check_work, hall_check_func);

static void hall_check_func(struct work_struct *work)
{
	input_event(hall_input, EV_SW, SW_LID, !gpio_get_value(HALL_GPIO_INT));
	input_sync(hall_input);
	return;
}

static irqreturn_t hall_irq_handler(int irq, void *dev_id)
{
	schedule_work(&hall_check_work);

	return IRQ_HANDLED;
}

static int hall_probe(struct platform_device *pdev)
{
	int err;
	hall_input = input_allocate_device();
	if (!hall_input) {
		dev_err(&pdev->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}
	hall_input->name = pdev->name;
	hall_input->phys = "proximity-sensor/hall";
	hall_input->id.bustype = BUS_HOST;
	hall_input->dev.parent = &pdev->dev;
	input_set_capability(hall_input, EV_SW, SW_LID);
	err = input_register_device(hall_input);
	if (err) {
		dev_err(&pdev->dev,
			"failed to register input device:%d\n", err);
		return err;
	}
	err = gpio_request(HALL_GPIO_INT, "hall_interrupt");
	if (err) {
		dev_err(&pdev->dev,
			"Fail to request interrupt gpio PIN %d.\n", HALL_GPIO_INT);
		return err;
	}	gpio_direction_input(HALL_GPIO_INT);
	err = request_irq(gpio_to_irq(HALL_GPIO_INT), hall_irq_handler, IRQ_TYPE_EDGE_BOTH | IRQF_NO_SUSPEND,
			  HALL_DRIVER_NAME, pdev);
	if (err) {
		printk(KERN_ERR "No interrupt for  wake GPIO: %i\n",
		       HALL_GPIO_INT);
		input_unregister_device(hall_input);
		return err;
	}
	return 0;
}

static int hall_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver hall_driver = {
	.probe		= hall_probe,
	.remove		= hall_remove,
	.driver	= {
		.name	= HALL_DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init hall_init(void)
{
	struct platform_device *pdev;
	int ret;

	pdev = platform_device_alloc(HALL_DRIVER_NAME, -1);
	if (!pdev) {
		pr_err("%s(): out of memory\n", __func__);
		return -ENOMEM;
	}

	ret = platform_device_add(pdev);
	ret = platform_driver_register(&hall_driver);
	if (ret < 0)
		printk(KERN_ERR "Fail to register ps stm8t143 driver\n");

	return ret;
}

static void __exit hall_exit(void)
{
	platform_driver_unregister(&hall_driver);
}


module_init(hall_init);
module_exit(hall_exit);


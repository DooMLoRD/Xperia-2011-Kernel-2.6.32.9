#include <linux/zeus_esd.h>
#include <linux/interrupt.h>

/* To simplify life */
#define ACCESS_PDATA(x)		((struct platform_esd_data*)(x)->dev.platform_data)

struct esd_device {
	struct platform_device *pdev;
};

/**
 * Called whenever the two GPIO's trigger
 *
 */
static irqreturn_t zeus_esd_detect(int irq, void *data)
{
	struct esd_device* dev = data;

	/* Sanity! */
	if (data != NULL && ACCESS_PDATA(dev->pdev)) 
	{
		bool signal = false;
		
		/* See which IRQ that triggered and log it */
		if (ACCESS_PDATA(dev->pdev)->positive == irq) {
			dev_err(&dev->pdev->dev, "Positive ESD detected\n");
			signal = true;
		} else if (ACCESS_PDATA(dev->pdev)->negative == irq) {
			dev_err(&dev->pdev->dev, "Negative ESD detected\n");
			signal = true;
		} else
			dev_err(&dev->pdev->dev, "Interrupt triggered but no valid IRQ\n");


		if (signal)
		{
			/* Signal the subscribers (if any) */
			eventESD_t* i;

			for (i = ACCESS_PDATA(dev->pdev)->eventESD; *i != NULL; ++i) {
				dev_err(&dev->pdev->dev, "Signaling subscriber\n");
				(*i)();
			}
		}
	}
	
	return IRQ_HANDLED;
}

static int __init zeus_esd_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct platform_esd_data *plat = pdev->dev.platform_data;
	struct esd_device* esddev = NULL;

	/* check device validity */
	/* must have platform data */
	if (!plat) {
		dev_err(&pdev->dev, "Platform data not available\n");
		rc = -EINVAL;
		goto out;
	}

	dev_err(&pdev->dev, "Hello there, probing hardware!\n");
	
	rc = -ENODEV;

	esddev = kzalloc(sizeof(struct esd_device), GFP_KERNEL);
	if (!esddev) {
		dev_err(&pdev->dev, 
		        "Failed to allocate memory for device\n");
		rc = -ENOMEM;
		goto out;
	}

	/* cross links */
	esddev->pdev = pdev;
	platform_set_drvdata(pdev, esddev);

	/* Initialize the GPIOs */
	ACCESS_PDATA(esddev->pdev)->init();
	
	/* Allocate interrupt handlers for this */
	if (ACCESS_PDATA(esddev->pdev)->positive != 0) {
		rc = request_irq(ACCESS_PDATA(esddev->pdev)->positive, 
		                 zeus_esd_detect, 
		                 ACCESS_PDATA(esddev->pdev)->positive_detect, 
		                 "zeus-esd", 
		                 esddev);
		if (rc < 0)
		{
			dev_err(&pdev->dev, 
			        "Unable to get IRQ for positive ESD\n");
			goto cleanupmem;
		}
	}
	
	if (ACCESS_PDATA(esddev->pdev)->negative != 0) {
		rc = request_irq(ACCESS_PDATA(esddev->pdev)->negative, 
		                 zeus_esd_detect, 
		                 ACCESS_PDATA(esddev->pdev)->negative_detect, 
		                 "zeus-esd", 
		                 esddev);
		if (rc < 0)
		{
			dev_err(&pdev->dev, 
			        "Unable to get IRQ for negative ESD\n");
			goto cleanupirq1;
		}
	}

out:
	return rc;

	/**
	 * Error handling area... Don't really like this "goto" business
	 * but it's A solution, just not THE solution.
	 */
/**
 * This one is not used yet, might be if we ever get more complicated.
 */
/*
cleanupirq2:
	free_irq(ACCESS_PDATA(esddev->pdev)->negative, NULL);
 */
cleanupirq1:
	if (ACCESS_PDATA(esddev->pdev)->positive != 0)
		free_irq(ACCESS_PDATA(esddev->pdev)->positive, NULL);
cleanupmem:
	dev_info(&pdev->dev, "Unload\n");
	kfree(esddev);
	return rc;	
}

static int __devexit zeus_esd_remove(struct platform_device *pdev)
{
	struct esd_device *esddev = platform_get_drvdata(pdev);

	if (esddev != NULL) {
		if (ACCESS_PDATA(esddev->pdev)->negative)
			free_irq(ACCESS_PDATA(esddev->pdev)->negative, NULL);
		if (ACCESS_PDATA(esddev->pdev)->positive)
			free_irq(ACCESS_PDATA(esddev->pdev)->positive, NULL);

		dev_info(&pdev->dev, "Unload\n");
		kfree(esddev);
	}

	return 0;
}


static struct platform_driver zeus_esd_driver = {
	.probe          = zeus_esd_probe,
	.remove         = __devexit_p(zeus_esd_remove),
	
	.driver         = {
		.name   = "zeus_esd",
	},
};

static int __init mod_init(void)
{
	int rc = platform_driver_register(&zeus_esd_driver);
	if (rc)
		pr_err("platform_driver_register failed: %d\n", rc);
	return rc;
}

static void __exit mod_exit(void)
{
	platform_driver_unregister(&zeus_esd_driver);
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_DESCRIPTION("Zeus ESD detection device");
MODULE_LICENSE("GPL");


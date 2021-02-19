#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>
#include <drivers/led_strip.h>
#include <drivers/pwm.h>
#include <stdio.h>
#include <sys/util.h>
#include <string.h>
#include <sys/printk.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

//#define PWM_DRIVER	DT_ALIAS_PWM_LED0_PWMS_CONTROLLER
#define PIN	DT_GPIO_PIN(DT_ALIAS(led0), gpios)
static struct device *led;

static const struct device *pwm1;
static const struct device *pwm2;

rcl_subscription_t pwm1_subscription;
rcl_subscription_t pwm2_subscription;

std_msgs__msg__Int32 pwm1_msg;
std_msgs__msg__Int32 pwm2_msg;

void pwm1_subscription_callback(const void * msg)
{
}

void pwm2_subscription_callback(const void * msg)
{
}

void main(void)
{
	led = device_get_binding(DT_GPIO_LABEL(DT_ALIAS(led0), gpios));
	gpio_pin_configure(led, PIN, GPIO_OUTPUT_ACTIVE);

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "bfr_hardware_node", "", &support));

	uint32_t pwm1_pinmux = DT_PROP(DT_ALIAS(pwm1), pinmux);
	uint32_t pwm2_pinmux = DT_PROP(DT_ALIAS(pwm2), pinmux);

	// ---- Devices configuration ----
	pwm1 = device_get_binding(DT_LABEL(DT_CHILD(DT_ALIAS(tim1), pwm)));
	pwm2 = device_get_binding(DT_LABEL(DT_CHILD(DT_ALIAS(tim2), pwm)));

	uint64_t cycles = 0;

	RCCHECK(rclc_subscription_init_default(&pwm1_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/hal/hardware/pwm1"));
	RCCHECK(rclc_subscription_init_default(&pwm2_subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/hal/hardware/pwm2"));

	// Creating a executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &pwm1_subscription, &pwm1_msg, &pwm1_subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &pwm2_subscription, &pwm2_msg, &pwm2_subscription_callback, ON_NEW_DATA));


	pwm_get_cycles_per_sec(pwm1, pwm1_pinmux, &cycles);
	//gpio_pin_set(led, PIN, 1);

	if (pwm_pin_set_usec(pwm1, pwm1_pinmux, 1000, 1000 / 2U, 0)) {
		printk("pwm pin set fails\n");
		return;
	}	

	while (1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		printk("Running once");
	}

	RCCHECK(rcl_node_fini(&node));
}
controller = get_controller("controller")
write(controller, "digital_out[2]", False)
wait_for_io(controller, "digital_out[2]", False)
wait_for_bool_io(controller, "digital_out[2]", False)
text = "Hello World!"
text_replaced = replace(text, "World", "Wandelbots NOVA")

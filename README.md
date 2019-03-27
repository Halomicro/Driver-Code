# Driver-Code

HL6111R V0.1

	2019/03/27
	Write the following two functions：
	1、The Vout configuration function is：static int hl6111r_set_vout_regulation(struct hl6111r_device_info *di)
    　　Input:1)hl6111r_device_info－＞VOUT_RNG_SEL，The corresponding register is REG30<7:6>.
              2)hl6111r_device_info－＞VOUT_TARGET, The corresponding register is REG0E,unit is the mV.

	2、The Vout read function is:static int hl6111r_get_vout_regulation(struct hl6111r_device_info *di),
        Output:hl6111r_device_info－＞vout_read_val,the output is the voltage in units of uV.
	In additiont:Please refer to the latest datasheet for the IIC address of HL6111R.

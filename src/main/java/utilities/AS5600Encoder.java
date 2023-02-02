// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package utilities;
import com.ctre.phoenixpro.signals.IsPROLicensedValue;

import edu.wpi.first.wpilibj.I2C;

/** Add your docs here. */
public class AS5600Encoder {
    public I2C i2c;

    public static final class Registers {
        // Config.
        public static final int ZMCO = 0x00;
        public static final int []ZPOS = {0x01, 0x02};
        public static final int []MPOS = {0x03, 0x04};
        public static final int []MANG = {0x05, 0x06};
        public static final int []CONF = {0x07, 0x08};

        // Output.
        public static final int []RAW_ANGLE = {0x0c, 0x0d};
        public static final int []ANGLE = {0x0e, 0x0f};

        // Status.
        public static final int STATUS = 0x0B;
        public static final int AGC = 0x1a;
        public static final int []MAGNITUDE = {0x1b, 0x1c};

        // Burn.
        public static final int BURN = 0xff;
    }

    public AS5600Encoder(I2C.Port port, int deviceAddress) {
        i2c = new I2C(port, deviceAddress);
    }

    public void test() {
        System.out.println(String.format("byte read: %d", getAngle()));
    }

    public int getAngle() {
        /*
        byte []bytes = new byte [1];

        if (i2c.read(Registers.ANGLE, 1, bytes)) {
            return 0;
        }

        return (int)bytes[0];
        */
        return 0;
    }

    // Some values use 2 registers for 16 bit.
    private int getIntFromRegisterGroup(int []addressList) {
        int i;
        byte []byteList = new byte[2];

        for (i = 0; i < byteList.length; i++) {
            byteList[i] = getByteFromRegister(addressList[i]);
        }

        return to12BitFormat(byteList);
    }

    private byte getByteFromRegister(int address) {
        byte []value = new byte[1];

        if (i2c.read(address, 1, value)) {
            return 0x0; // Error.
        }

        return value[0];
    }

    private static int to12BitFormat(byte[] bytes) {
        return bytes[0] << 8 | bytes[1];
    }

    public I2C getI2C() {
        return i2c;
    }
}

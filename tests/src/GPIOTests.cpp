//
//  GPIOTests.cpp
//  Bedrock Tests
//
//  Created by Eric Yanush on 2015-12-19.
//  Copyright © 2015 EricYanush. All rights reserved.
//

#include "gtest/gtest.h"
#include "GPIO.hpp"

class GPIOTest : public ::testing::Test {
    
protected:
    
    virtual void SetUp() {
    }
    
    virtual void TearDown() {
    }
};

TEST_F(GPIOTest, TestGPIOPortLayout) {
    ASSERT_EQ(0x00, offsetof(GPIOPort, MODE));
    ASSERT_EQ(0x04, offsetof(GPIOPort, OTYPE));
    ASSERT_EQ(0x08, offsetof(GPIOPort, OSPEED));
    ASSERT_EQ(0x0C, offsetof(GPIOPort, PUPD));
    ASSERT_EQ(0x10, offsetof(GPIOPort, IDR));
    ASSERT_EQ(0x14, offsetof(GPIOPort, ODR));
    ASSERT_EQ(0x18, offsetof(GPIOPort, BSR));
    ASSERT_EQ(0x1C, offsetof(GPIOPort, LCK));
    ASSERT_EQ(0x20, offsetof(GPIOPort, AFL));
    ASSERT_EQ(0x24, offsetof(GPIOPort, AFH));
    ASSERT_EQ(0x28, offsetof(GPIOPort, BR));
}

TEST_F(GPIOTest, TestModeTypes) {
    ASSERT_EQ(0b00, static_cast<uint32_t>(PinMode::input));
    ASSERT_EQ(0b01, static_cast<uint32_t>(PinMode::output));
    ASSERT_EQ(0b10, static_cast<uint32_t>(PinMode::alternateFunc));
    ASSERT_EQ(0b11, static_cast<uint32_t>(PinMode::analog));
}

TEST_F(GPIOTest, TestOuputSpeedTypes) {
    ASSERT_EQ(0b00, static_cast<uint32_t>(OutputSpeed::low));
    ASSERT_EQ(0b01, static_cast<uint32_t>(OutputSpeed::medium));
    ASSERT_EQ(0b11, static_cast<uint32_t>(OutputSpeed::high));
}

TEST_F(GPIOTest, TestIOPullTypes) {
    ASSERT_EQ(0b00, static_cast<uint32_t>(IOPullType::none));
    ASSERT_EQ(0b01, static_cast<uint32_t>(IOPullType::up));
    ASSERT_EQ(0b10, static_cast<uint32_t>(IOPullType::down));
}

class GPIOPinTest : public ::testing::Test {
    
protected:
    virtual void SetUp() {
    }
    
    virtual void TearDown() {
    }
    
    static GPIOPort testPort;
    static GPIOPort* fakePort() {
        return &testPort;
    }
    
    GPIOPin<fakePort, 0> zero;
    GPIOPin<fakePort, 1> one;
    GPIOPin<fakePort, 2> two;
    GPIOPin<fakePort, 3> three;
    GPIOPin<fakePort, 4> four;
    GPIOPin<fakePort, 5> five;
    GPIOPin<fakePort, 6> six;
    GPIOPin<fakePort, 7> seven;
    GPIOPin<fakePort, 8> eight;
    GPIOPin<fakePort, 9> nine;
    GPIOPin<fakePort, 10> ten;
    GPIOPin<fakePort, 11> eleven;
    GPIOPin<fakePort, 12> twelve;
    GPIOPin<fakePort, 13> thirteen;
    GPIOPin<fakePort, 14> fourteen;
    GPIOPin<fakePort, 15> fifteen;
};
 GPIOPort GPIOPinTest::testPort;


TEST_F(GPIOPinTest, TestSetMode) {
    
    testPort.MODE = 0;
    zero.setMode(PinMode::analog);
    ASSERT_EQ(static_cast<uint32_t>(PinMode::analog) << 0, testPort.MODE);
    //Ensure we handle changes correctly, and zero out the mode for the port on change
    zero.setMode(PinMode::output);
    ASSERT_EQ(static_cast<uint32_t>(PinMode::output), testPort.MODE);
    
    testPort.MODE = 0;
    one.setMode(PinMode::analog);
    ASSERT_EQ(static_cast<uint32_t>(PinMode::analog) << 2, testPort.MODE);
    
    testPort.MODE = 0;
    seven.setMode(PinMode::analog);
    ASSERT_EQ(static_cast<uint32_t>(PinMode::analog) << 14, testPort.MODE);
    
    testPort.MODE = 0;
    fourteen.setMode(PinMode::analog);
    ASSERT_EQ(static_cast<uint32_t>(PinMode::analog) << 28, testPort.MODE);
    
    testPort.MODE = 0;
    fifteen.setMode(PinMode::analog);
    ASSERT_EQ(static_cast<uint32_t>(PinMode::analog) << 30, testPort.MODE);
}

TEST_F(GPIOPinTest, TestSetOutputSpeed) {
    testPort.OSPEED = 0;
    zero.setOutSpeed(OutputSpeed::medium);
    ASSERT_EQ(static_cast<uint32_t>(OutputSpeed::medium), testPort.OSPEED);
    
    testPort.OSPEED = 0;
    one.setOutSpeed(OutputSpeed::high);
    ASSERT_EQ(static_cast<uint32_t>(OutputSpeed::high) << 2, testPort.OSPEED);
    
    testPort.OSPEED = 0;
    //ensure we correctly clear bits
    seven.setOutSpeed(OutputSpeed::high);
    seven.setOutSpeed(OutputSpeed::low);
    ASSERT_EQ(static_cast<uint32_t>(OutputSpeed::low) << 14, testPort.OSPEED);
    
    testPort.OSPEED = 0;
    fourteen.setOutSpeed(OutputSpeed::medium);
    ASSERT_EQ(static_cast<uint32_t>(OutputSpeed::medium) << 28, testPort.OSPEED);
    
    testPort.OSPEED = 0;
    fifteen.setOutSpeed(OutputSpeed::high);
    ASSERT_EQ(static_cast<uint32_t>(OutputSpeed::high) << 30, testPort.OSPEED);
}

TEST_F(GPIOPinTest, TestSetOutputType) {
    testPort.OTYPE = 0;
    zero.setOutputType(OutputType::opendrain);
    ASSERT_EQ(static_cast<uint32_t>(OutputType::opendrain), testPort.OTYPE);
    
    testPort.OTYPE = 0;
    one.setOutputType(OutputType::opendrain);
    ASSERT_EQ(static_cast<uint32_t>(OutputType::opendrain) << 1, testPort.OTYPE);
    
    testPort.OTYPE = 0;
    //ensure we correctly clear bits
    seven.setOutputType(OutputType::opendrain);
    seven.setOutputType(OutputType::pushpull);
    ASSERT_EQ(static_cast<uint32_t>(OutputType::pushpull) << 7, testPort.OTYPE);
    
    testPort.OTYPE = 0;
    fourteen.setOutputType(OutputType::opendrain);
    ASSERT_EQ(static_cast<uint32_t>(OutputType::opendrain) << 14, testPort.OTYPE);
    
    testPort.OTYPE = 0;
    fifteen.setOutputType(OutputType::opendrain);
    ASSERT_EQ(static_cast<uint32_t>(OutputType::opendrain) << 15, testPort.OTYPE);
}

TEST_F(GPIOPinTest, TestSetPullType) {
    testPort.PUPD = 0;
    zero.setPullType(IOPullType::down);
    ASSERT_EQ(static_cast<uint32_t>(IOPullType::down), testPort.PUPD);
    
    testPort.PUPD = 0;
    one.setPullType(IOPullType::up);
    ASSERT_EQ(static_cast<uint32_t>(IOPullType::up) << 2, testPort.PUPD);
    
    testPort.PUPD = 0;
    //ensure we correctly clear bits
    seven.setPullType(IOPullType::up);
    seven.setPullType(IOPullType::none);
    ASSERT_EQ(static_cast<uint32_t>(IOPullType::none) << 14, testPort.PUPD);
    
    testPort.PUPD = 0;
    fourteen.setPullType(IOPullType::down);
    ASSERT_EQ(static_cast<uint32_t>(IOPullType::down) << 28, testPort.PUPD);
    
    testPort.PUPD = 0;
    fifteen.setPullType(IOPullType::down);
    ASSERT_EQ(static_cast<uint32_t>(IOPullType::down) << 30, testPort.PUPD);
}

TEST_F(GPIOPinTest, TestSetOn) {
    testPort.ODR = 0;
    zero.on();
    ASSERT_EQ(1, testPort.ODR);
    
    testPort.ODR = 0;
    seven.on();
    ASSERT_EQ(1 << 7, testPort.ODR);
    
    testPort.ODR = 0;
    fifteen.on();
    ASSERT_EQ(1 << 15, testPort.ODR);
}

TEST_F(GPIOPinTest, TestSetOff) {
    testPort.ODR = 1;
    zero.off();
    ASSERT_EQ(0, testPort.ODR);
    
    testPort.ODR = 1 << 8;
    eight.off();
    ASSERT_EQ(0, testPort.ODR);
    
    testPort.ODR = 1 << 15;
    fifteen.off();
    ASSERT_EQ(0, testPort.ODR);
}

TEST_F(GPIOPinTest, TestToggle) {
    testPort.ODR = 1;
    zero.toggle();
    ASSERT_EQ(0, testPort.ODR);
    zero.toggle();
    ASSERT_EQ(1, testPort.ODR);
    
    testPort.ODR = 1 << 8;
    eight.toggle();
    ASSERT_EQ(0, testPort.ODR);
    eight.toggle();
    ASSERT_EQ(1 << 8, testPort.ODR);
    
    testPort.ODR = 1 << 15;
    fifteen.toggle();
    ASSERT_EQ(0, testPort.ODR);
    fifteen.toggle();
    ASSERT_EQ(1 << 15, testPort.ODR);
}

TEST_F(GPIOPinTest, TestIsOn) {
    testPort.ODR = 0;
    ASSERT_EQ(false, zero.isOn());
    testPort.ODR = 1;
    ASSERT_EQ(true, zero.isOn());
    
    testPort.ODR = 0;
    ASSERT_EQ(false, seven.isOn());
    testPort.ODR = (1 << 7);
    ASSERT_EQ(true, seven.isOn());
    
    testPort.ODR = 0;
    ASSERT_EQ(false, fifteen.isOn());
    testPort.ODR = (1 << 15);
    ASSERT_EQ(true, fifteen.isOn());
}
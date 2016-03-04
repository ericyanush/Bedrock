//
//  GPIOTests.cpp
//  Bedrock Tests
//
//  Created by Eric Yanush on 2015-12-19.
//  Copyright © 2015 EricYanush. All rights reserved.
//

#include "gtest/gtest.h"
#include "GPIO.hpp"

using namespace Bedrock;

class GPIOTest : public ::testing::Test {
    
protected:
    
    virtual void SetUp() {
    }
    
    virtual void TearDown() {
    }
};

TEST_F(GPIOTest, TestGPIOPortLayout) {
    ASSERT_EQ(0x00, offsetof(GPIO::GPIOPort, MODE));
    ASSERT_EQ(0x04, offsetof(GPIO::GPIOPort, OTYPE));
    ASSERT_EQ(0x08, offsetof(GPIO::GPIOPort, OSPEED));
    ASSERT_EQ(0x0C, offsetof(GPIO::GPIOPort, PUPD));
    ASSERT_EQ(0x10, offsetof(GPIO::GPIOPort, IDR));
    ASSERT_EQ(0x14, offsetof(GPIO::GPIOPort, ODR));
    ASSERT_EQ(0x18, offsetof(GPIO::GPIOPort, BSR));
    ASSERT_EQ(0x1C, offsetof(GPIO::GPIOPort, LCK));
    ASSERT_EQ(0x20, offsetof(GPIO::GPIOPort, AFL));
    ASSERT_EQ(0x24, offsetof(GPIO::GPIOPort, AFH));
    ASSERT_EQ(0x28, offsetof(GPIO::GPIOPort, BR));
}

TEST_F(GPIOTest, TestModeTypes) {
    ASSERT_EQ(0b00, static_cast<uint32_t>(GPIO::PinMode::input));
    ASSERT_EQ(0b01, static_cast<uint32_t>(GPIO::PinMode::output));
    ASSERT_EQ(0b10, static_cast<uint32_t>(GPIO::PinMode::alternateFunc));
    ASSERT_EQ(0b11, static_cast<uint32_t>(GPIO::PinMode::analog));
}

TEST_F(GPIOTest, TestOuputSpeedTypes) {
    ASSERT_EQ(0b00, static_cast<uint32_t>(GPIO::OutputSpeed::low));
    ASSERT_EQ(0b01, static_cast<uint32_t>(GPIO::OutputSpeed::medium));
    ASSERT_EQ(0b11, static_cast<uint32_t>(GPIO::OutputSpeed::high));
}

TEST_F(GPIOTest, TestIOPullTypes) {
    ASSERT_EQ(0b00, static_cast<uint32_t>(GPIO::IOPullType::none));
    ASSERT_EQ(0b01, static_cast<uint32_t>(GPIO::IOPullType::up));
    ASSERT_EQ(0b10, static_cast<uint32_t>(GPIO::IOPullType::down));
}

TEST_F(GPIOTest, TestAlternateFunctinonTypes) {
    ASSERT_EQ(0, static_cast<uint32_t>(GPIO::AlternateFunction::AF0));
    ASSERT_EQ(1, static_cast<uint32_t>(GPIO::AlternateFunction::AF1));
    ASSERT_EQ(2, static_cast<uint32_t>(GPIO::AlternateFunction::AF2));
    ASSERT_EQ(3, static_cast<uint32_t>(GPIO::AlternateFunction::AF3));
    ASSERT_EQ(4, static_cast<uint32_t>(GPIO::AlternateFunction::AF4));
    ASSERT_EQ(5, static_cast<uint32_t>(GPIO::AlternateFunction::AF5));
    ASSERT_EQ(6, static_cast<uint32_t>(GPIO::AlternateFunction::AF6));
    ASSERT_EQ(7, static_cast<uint32_t>(GPIO::AlternateFunction::AF7));
    ASSERT_EQ(8, static_cast<uint32_t>(GPIO::AlternateFunction::AF8));
    ASSERT_EQ(9, static_cast<uint32_t>(GPIO::AlternateFunction::AF9));
    ASSERT_EQ(10, static_cast<uint32_t>(GPIO::AlternateFunction::AF10));
    ASSERT_EQ(11, static_cast<uint32_t>(GPIO::AlternateFunction::AF11));
    ASSERT_EQ(12, static_cast<uint32_t>(GPIO::AlternateFunction::AF12));
    ASSERT_EQ(13, static_cast<uint32_t>(GPIO::AlternateFunction::AF13));
    ASSERT_EQ(14, static_cast<uint32_t>(GPIO::AlternateFunction::AF14));
    ASSERT_EQ(15, static_cast<uint32_t>(GPIO::AlternateFunction::AF15));
}

class GPIOPinTest : public ::testing::Test {
public:
    
    GPIOPinTest() :
                zero(fakePort, 0),
                one(fakePort, 1),
                two(fakePort, 2),
                three(fakePort, 3),
                four(fakePort, 4),
                five(fakePort, 5),
                six(fakePort, 6),
                seven(fakePort, 7),
                eight(fakePort, 8),
                nine(fakePort, 9),
                ten(fakePort, 10),
                eleven(fakePort, 11),
                twelve(fakePort, 12),
                thirteen(fakePort, 13),
                fourteen(fakePort, 14),
                fifteen(fakePort, 15)
    {  }
    
protected:
    virtual void SetUp() {
    }
    
    virtual void TearDown() {
    }
    
    static GPIO::GPIOPort testPort;
    static GPIO::GPIOPort& fakePort() {
        return testPort;
    }
    
    GPIO::GPIOPin zero;
    GPIO::GPIOPin one;
    GPIO::GPIOPin two;
    GPIO::GPIOPin three;
    GPIO::GPIOPin four;
    GPIO::GPIOPin five;
    GPIO::GPIOPin six;
    GPIO::GPIOPin seven;
    GPIO::GPIOPin eight;
    GPIO::GPIOPin nine;
    GPIO::GPIOPin ten;
    GPIO::GPIOPin eleven;
    GPIO::GPIOPin twelve;
    GPIO::GPIOPin thirteen;
    GPIO::GPIOPin fourteen;
    GPIO::GPIOPin fifteen;
};
 GPIO::GPIOPort GPIOPinTest::testPort;


TEST_F(GPIOPinTest, TestSetAlternateFunction)
{
    using AltFunc = Bedrock::GPIO::AlternateFunction;
    
    five.setAlternateFunction(AltFunc::AF11);
    ASSERT_EQ(AltFunc::AF11, static_cast<AltFunc>((testPort.AFL >> 20) & 0xF));
    
    eleven.setAlternateFunction(AltFunc::AF8);
    ASSERT_EQ(AltFunc::AF8, static_cast<AltFunc>((testPort.AFH >> 12) & 0xF));
}

TEST_F(GPIOPinTest, TestGetAlternateFunction)
{
    using AltFunc = Bedrock::GPIO::AlternateFunction;
    
    two.setAlternateFunction(AltFunc::AF6);
    ASSERT_EQ(AltFunc::AF6, two.getAlternateFunction());
    
    fourteen.setAlternateFunction(AltFunc::AF3);
    ASSERT_EQ(AltFunc::AF3, fourteen.getAlternateFunction());
}

TEST_F(GPIOPinTest, TestSetMode) {
    
    testPort.MODE = 0;
    zero.setMode(GPIO::PinMode::analog);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::PinMode::analog) << 0, testPort.MODE);
    //Ensure we handle changes correctly, and zero out the mode for the port on change
    zero.setMode(GPIO::PinMode::output);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::PinMode::output), testPort.MODE);
    
    testPort.MODE = 0;
    one.setMode(GPIO::PinMode::analog);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::PinMode::analog) << 2, testPort.MODE);
    
    testPort.MODE = 0;
    seven.setMode(GPIO::PinMode::analog);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::PinMode::analog) << 14, testPort.MODE);
    
    testPort.MODE = 0;
    fourteen.setMode(GPIO::PinMode::analog);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::PinMode::analog) << 28, testPort.MODE);
    
    testPort.MODE = 0;
    fifteen.setMode(GPIO::PinMode::analog);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::PinMode::analog) << 30, testPort.MODE);
}

TEST_F(GPIOPinTest, TestGetMode)
{
    using Mode = Bedrock::GPIO::PinMode;
    
    four.setMode(Mode::output);
    ASSERT_EQ(Mode::output, four.getMode());
    
    thirteen.setMode(Mode::alternateFunc);
    ASSERT_EQ(Mode::alternateFunc, thirteen.getMode());
}

TEST_F(GPIOPinTest, TestSetOutputSpeed) {
    testPort.OSPEED = 0;
    zero.setOutSpeed(GPIO::OutputSpeed::medium);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::OutputSpeed::medium), testPort.OSPEED);
    
    testPort.OSPEED = 0;
    one.setOutSpeed(GPIO::OutputSpeed::high);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::OutputSpeed::high) << 2, testPort.OSPEED);
    
    testPort.OSPEED = 0;
    //ensure we correctly clear bits
    seven.setOutSpeed(GPIO::OutputSpeed::high);
    seven.setOutSpeed(GPIO::OutputSpeed::low);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::OutputSpeed::low) << 14, testPort.OSPEED);
    
    testPort.OSPEED = 0;
    fourteen.setOutSpeed(GPIO::OutputSpeed::medium);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::OutputSpeed::medium) << 28, testPort.OSPEED);
    
    testPort.OSPEED = 0;
    fifteen.setOutSpeed(GPIO::OutputSpeed::high);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::OutputSpeed::high) << 30, testPort.OSPEED);
}

TEST_F(GPIOPinTest, TestGetOutputSpeed)
{
    using OutSpeed = Bedrock::GPIO::OutputSpeed;
    
    five.setOutSpeed(OutSpeed::medium);
    ASSERT_EQ(OutSpeed::medium, five.getOutSpeed());
    
    nine.setOutSpeed(OutSpeed::high);
    ASSERT_EQ(OutSpeed::high, nine.getOutSpeed());
}

TEST_F(GPIOPinTest, TestSetOutputType) {
    testPort.OTYPE = 0;
    zero.setOutputType(GPIO::OutputType::opendrain);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::OutputType::opendrain), testPort.OTYPE);
    
    testPort.OTYPE = 0;
    one.setOutputType(GPIO::OutputType::opendrain);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::OutputType::opendrain) << 1, testPort.OTYPE);
    
    testPort.OTYPE = 0;
    //ensure we correctly clear bits
    seven.setOutputType(GPIO::OutputType::opendrain);
    seven.setOutputType(GPIO::OutputType::pushpull);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::OutputType::pushpull) << 7, testPort.OTYPE);
    
    testPort.OTYPE = 0;
    fourteen.setOutputType(GPIO::OutputType::opendrain);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::OutputType::opendrain) << 14, testPort.OTYPE);
    
    testPort.OTYPE = 0;
    fifteen.setOutputType(GPIO::OutputType::opendrain);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::OutputType::opendrain) << 15, testPort.OTYPE);
}

TEST_F(GPIOPinTest, TestGetOutputType)
{
    using OutType = Bedrock::GPIO::OutputType;
    
    three.setOutputType(OutType::pushpull);
    ASSERT_EQ(OutType::pushpull, three.getOutputType());
    
    fifteen.setOutputType(OutType::opendrain);
    ASSERT_EQ(OutType::opendrain, fifteen.getOutputType());
}

TEST_F(GPIOPinTest, TestSetPullType) {
    testPort.PUPD = 0;
    zero.setPullType(GPIO::IOPullType::down);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::IOPullType::down), testPort.PUPD);
    
    testPort.PUPD = 0;
    one.setPullType(GPIO::IOPullType::up);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::IOPullType::up) << 2, testPort.PUPD);
    
    testPort.PUPD = 0;
    //ensure we correctly clear bits
    seven.setPullType(GPIO::IOPullType::up);
    seven.setPullType(GPIO::IOPullType::none);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::IOPullType::none) << 14, testPort.PUPD);
    
    testPort.PUPD = 0;
    fourteen.setPullType(GPIO::IOPullType::down);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::IOPullType::down) << 28, testPort.PUPD);
    
    testPort.PUPD = 0;
    fifteen.setPullType(GPIO::IOPullType::down);
    ASSERT_EQ(static_cast<uint32_t>(GPIO::IOPullType::down) << 30, testPort.PUPD);
}

TEST_F(GPIOPinTest, TestGetPullType)
{
    using Pull = Bedrock::GPIO::IOPullType;
    
    two.setPullType(Pull::up);
    ASSERT_EQ(Pull::up, two.getPullType());
    
    eleven.setPullType(Pull::down);
    ASSERT_EQ(Pull::down, eleven.getPullType());
    
    eight.setPullType(Pull::none);
    ASSERT_EQ(Pull::none, eight.getPullType());
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
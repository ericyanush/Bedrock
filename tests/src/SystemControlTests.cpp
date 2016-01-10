//
//  SystemControlTests.cpp
//  Bedrock
//
//  Created by Eric Yanush on 2016-01-07.
//  Copyright © 2016 EricYanush. All rights reserved.
//

#include "SystemControl.hpp"
#include "gtest/gtest.h"

class SystemControlTest : public ::testing::Test {
    
protected:
    
    virtual void SetUp() {
    }
    
    virtual void TearDown() {
    }
};


TEST_F(SystemControlTest, TestRegisterLayout) {
    ASSERT_EQ(0x00, offsetof(SystemControl, CPUID));
    ASSERT_EQ(0x04, offsetof(SystemControl, ICSR));
    ASSERT_EQ(0x08, offsetof(SystemControl, VTOR));
    ASSERT_EQ(0x0C, offsetof(SystemControl, AIRCR));
    ASSERT_EQ(0x10, offsetof(SystemControl, SCR));
    ASSERT_EQ(0x14, offsetof(SystemControl, CCR));
    ASSERT_EQ(0x18, offsetof(SystemControl, SHPR1));
    ASSERT_EQ(0x1C, offsetof(SystemControl, SHPR2));
    ASSERT_EQ(0x20, offsetof(SystemControl, SHPR3));
    ASSERT_EQ(0x24, offsetof(SystemControl, SHCRS));
    ASSERT_EQ(0x28, offsetof(SystemControl, CFSR));
    ASSERT_EQ(0x2C, offsetof(SystemControl, HFSR));
    ASSERT_EQ(0x34, offsetof(SystemControl, MMAR));
    ASSERT_EQ(0x38, offsetof(SystemControl, BFAR));
    ASSERT_EQ(0x3C, offsetof(SystemControl, AFSR));
}

TEST_F(SystemControlTest, TestSetVectorTableAddress) {
    SystemControl control{0};
    const SystemControl untouched{0};
    
    control.setVectorBaseAddress(0xABCDEF01);
    ASSERT_EQ(0xABCDEF01, control.VTOR);
    control.VTOR = 0;
    ASSERT_EQ(0, memcmp(&untouched, &control, sizeof(SystemControl)));
}

TEST_F(SystemControlTest, TestGetVectorTableAddress) {
    SystemControl control = {0};
    control.VTOR = 0xABC123FF;
    
    ASSERT_EQ(0xABC123FF, control.getVectorBaseAddress());
}

TEST_F(SystemControlTest, TestSetPriorityGroups) {
    SystemControl control{0};
    const SystemControl untouched{0};
    
    control.setPriorityGroups(PriorityGroupCount::Groups_8);
    ASSERT_EQ(static_cast<uint32_t>(PriorityGroupCount::Groups_8),
              (control.AIRCR >> 8) & 0b111); //Test the appropriate group value was set
    ASSERT_EQ(0x05FA0000, control.AIRCR & 0xFFFF0000); // Ensure the VECT_KEY was written to enable the modification
    control.AIRCR = 0;
    ASSERT_EQ(0, memcmp(&untouched, &control, sizeof(SystemControl)));
}

TEST_F(SystemControlTest, TestGetPriorityGroups) {
    SystemControl control{0};
    control.setPriorityGroups(PriorityGroupCount::Groups_4);
    
    ASSERT_EQ(PriorityGroupCount::Groups_4, control.getPriorityGroups());
}
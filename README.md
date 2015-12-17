# Bedrock
[![Build Status](https://travis-ci.org/ericyanush/Bedrock.svg)](https://travis-ci.org/ericyanush/Bedrock)

C++ foundation framework for STM32
-----

## What is it?

Bedrock is a spin-off project that I am building to support my Engineering capstone project. Bedrock is a simple to use, lightweight, C++ framework for interacting with STM32 micro-controllers in a bare-metal environment.

## Why build _another_ hardware library

During the course of implementing my project, I found that I was not happy with any of the options that currently exist. I found ST's STM32Cube hardware library hard to test against and fraught with bugs, making it a poor choice for a project whose primary goal is stability and reliability. The mBed library is, behind the scenes, using STM32Cube, and its general support of ST chips is lacking compared to the NXP lines. libopencm3, while almost exclusively supporting ST devices, still didn't feel quite right. So, that leaves starting from scratch!

## Design Goals of Bedrock

1. *_Everything_* is testable. The interfaces to the hardware peripherals are implemented so that every interaction with a hardware register is verifiable.
2. *_Everything_* is tested. What's the point of making everything testable, if it hasn't been tested?
3. *High performance*. The hardware peripherals are modeled using zero-cost c++ classes which allow for abstractions and simple to use interfaces, without sacrificing raw performance.
4. *Control*. Nothing about how the hardware peripherals is hidden, allowing programmers to dig in and take full control of the hardware, when necessary, without having to code their own interfaces.
5. *Modern*. One issue I have always run into when doing bare-metal coding is the lack of support for modern compilers and language features. Bedrock supports being built with GCC4.9+ as well as clang/llvm, ensuring you can always use the latest compiler features and optimizations for your bare-metal ARM project.

## Status

As my project is currently targeting the STM32F303RE device, the STM32F303 family is currently all that is supported. However, most ST chips, especially the STM32F1 family are very similar, and this could very easily be ported/extrapolated to them as well.

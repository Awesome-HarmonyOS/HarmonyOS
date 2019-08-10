[![Build Status](https://travis-ci.org/LiteOS/LiteOS.svg?branch=develop)](https://travis-ci.org/LiteOS/LiteOS)

## Huawei LiteOS简介

Huawei LiteOS是华为面向物联网领域开发的一个基于实时内核的轻量级操作系统。本项目属于华为物联网操作系统[Huawei LiteOS](http://developer.huawei.com/ict/cn/site-iot/product/liteos)源码，现有基础内核支持任务管理、内存管理、时间管理、通信机制、中断管理、队列管理、事件管理、定时器等操作系统基础组件，更好地支持低功耗场景，支持tickless机制，支持定时器对齐。

同时提供端云协同能力，集成了LwM2M、CoAP、mbedtls、LwIP全套IoT互联协议栈，且在LwM2M的基础上，提供了AgentTiny模块，用户只需关注自身的应用，而不必关注LwM2M实现细节，直接使用AgentTiny封装的接口即可简单快速实现与云平台安全可靠的连接。

Huawei LiteOS自开源社区发布以来，围绕NB-IoT物联网市场从技术、生态、解决方案、商用支持等多维度使能合作伙伴，构建开源的物联网生态,目前已经聚合了30+ MCU和解决方案合作伙伴，共同推出一批开源开发套件和行业解决方案，帮助众多行业客户快速的推出物联网终端和服务，客户涵盖抄表、停车、路灯、环保、共享单车、物流等众多行业，为开发者提供 “一站式” 完整软件平台，有效降低开发门槛、缩短开发周期。

## LiteOS 代码导读

- [LiteOS内核源代码目录说明](./doc/LiteOS_Code_Info.md)

该文档描述的是LiteOS内核源代码的详细信息。通过此文档读者可以了解LiteOS的源代码结构，以及LiteOS的main()函数的功能。


## LiteOS 开发指南

[LiteOS开发指南](./doc/Huawei_LiteOS_Developer_Guide_zh.md)  

[LiteOS移植指南](https://liteos.github.io/porting/)  

该文档详细讲解了LiteOS各模块开发及其实现原理。用户可以根据该文档学习各模块的使用。


## LiteOS 接入云平台开发指南

* [LiteOS接入华为云平台](https://github.com/SuYai/OceanConnectHelp)
  * [LiteOS SDK端云互通组件开发指南](./doc/Huawei_LiteOS_SDK_Developer_Guide.md)

LiteOS SDK是Huawei LiteOS软件开发工具包（Software Development Kit），通过LiteOS SDK端云互通组件，简单快速地实现与华为 OceanConnect IoT平台安全可靠连接，可以大大减少开发周期，快速构建IoT产品。

LiteOS SDK是Huawei LiteOS软件开发工具包（Software Development Kit），通过LiteOS SDK端云互通组件，简单快速地实现与华为 OceanConnect IoT平台安全可靠连接，可以大大减少开发周期，快速构建IoT产品。
* [LiteOS SDK端云互通组件Coap开发指南](./doc/Huawei_LiteOS_SDK_Coap_LwM2M_Developer_Guide_zh.md)
* [LiteOS SDK端云互通组件MQTT开发指南](./doc/Huawei_LiteOS_SDK_MQTT_Developer_Guide.md)
  

* [LiteOS接入3rd云平台](https://github.com/LiteOS/LiteOS_Connect_to_3rd_Cloud)


## LiteOS 支持的硬件

* LiteOS开源项目目前支持ARM Cortex-M0，Cortex-M3，Cortex-M4，Cortex-M7等芯片架构

* [LiteOS支持的开发板列表](./doc/LiteOS_Supported_board_list.md)
Huawei LiteOS 联合业界主流MCU厂家，通过开发者活动，目前已经适配了30+ 通用 MCU开发套件，5套NB-IoT集成开发套件


## 开源协议

* 遵循BSD-3开源许可协议
* [Huawei LiteOS 知识产权政策](https://support.huaweicloud.com/productdesc-LiteOS/zh-cn_topic_0145347224.html)

## LiteOS Git入门必读

- [LiteOS Commit Message规则](./doc/LiteOS_Commit_Message.md)

该文档描述如何提交commit到LiteOS仓库，这是LiteOS开发必须遵守的commit规则，否则提交的commit会被驳回。请点链接了解详细信息。

- [Huawei LiteOS代码贡献流程](./doc/LiteOS_Contribute_Guide_GitGUI.md)

该文档描述开发者如何创建自己的仓库，开发然后贡献代码到LiteOS仓库。请点链接了解详细信息。


## 加入我们
* 欢迎提交issue对关心的问题发起讨论，欢迎提交PR参与特性建设
* 如您有合作意向，希望加入Huawei LiteOS生态合作伙伴，请发邮件至liteos@huawei.com，或访问[LiteOS官网](http://www.huawei.com/liteos)，进一步了解详细信息


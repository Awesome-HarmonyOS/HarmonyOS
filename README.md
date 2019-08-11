
[English Docs](README.md)  |  [中文文档](doc/README-zh.md)


# HarmonyOS
![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg) 

<p align="center">
  <a href="https://github.com/Awesome-HarmonyOS/HarmonyOS">
    <img src="assets/img/hi.jpg" width="750px">
  </a>
</p>



> A curated list of awesome things related to HarmonyOS


# Ⅰ . What is HarmonyOS?
`HormonyOS` is the first full-scene distributed OS based on micro-kernel. It is an operating system independently developed by Huawei. On August 9, 2019, `HormonyOS` System was officially released at the Huawei Developers Conference (<HDC.2019>). Huawei will take the lead in deploying intelligent terminals such as smart screens, vehicle terminals and wearable terminals. In the future, more and more intelligent devices will use open source `HormonyOS` .

`HormonyOS` achieves modular coupling, which corresponds to flexible deployment of different devices. `HormonyOS` has three layers of architecture. The first layer is the core, the second layer is the basic services, and the third layer is the program framework. It can be used in large screen, PC, automobile and other different equipment. It can also be used on mobile phones at any time, but for the time being, Huawei still prefers Android.

The underlying layer of HormonyOS is composed of  `HormonyOS micro kernel`, `Linux kernel` and `Lite OS` and it will become a complete hongmeng micro kernel architecture in the future.

# Ⅱ . Development process

- [2012] HarmonyOS started out in Shanghai’s Jiao Tong University in 2012. 
- [2018.8.24] Huawei applied for the `Huawei HormonyOS` trademark. The registration announcement date of `HormonyOS` trademark is May 14, 2019. The exclusive rights of `HormonyOS` trademark are from May 14, 2019 to May 13, 2029.
- [2019] Huawei has allegedly shipped 1 million smartphones with its self-developed "HarmonyOS" operating system onboard for testing.
- [2019.5.17] Huawei Operating System Team developed its own proprietary operating system `HormonyOS`.
- [2019.8.9]  Huawei officially released `HormonyOS`, and `HormonyOS` Operating System will be open source.

# Ⅲ . Features
![](assets/img/harmony/features.png)

HormonyOS is designed to meet the high standard connection requirements of full-scene intelligent experience. For this reason, Huawei has proposed four system solutions with major features.

* #### 1. Distributed architecture is first used in terminal OS to realize seamless collaborative experience across terminals

HormonyOS's "Distributed OS Architecture" and "Distributed Soft Bus Technology" shield the application developers from the difficulties of implementing the underlying technologies of the corresponding distributed applications through the four capabilities of public communication platform, distributed data management, distributed capability scheduling and virtual peripherals, enabling developers to focus on their own business logic, like open. Developing cross-terminal distributed applications like the same terminal also enables the final consumers to enjoy the seamless experience brought by the powerful cross-terminal business collaboration capability for each use scenario.

![](assets/img/harmony/harmonyos1.jpg)

* #### 2. Determine the time delay engine and high performance IPC technology to achieve natural fluency in the system

HormonyOS solves the problem of inadequate performance of existing systems by using two technologies: deterministic delay engine and high performance IPC. Determining the delay engine can assign priority and time limit of task execution in the system before task execution. The priority task resources will give priority to scheduling, and the application response delay will be reduced by 25.7%. The compact structure of Hongmeng microkernel greatly improves the performance of IPC (interprocess communication) and the efficiency of process communication is five times higher than that of existing systems.

![](assets/img/harmony/harmonyos2.jpg)

* #### 3. Rebuilding Trusted Security of Terminal Equipment Based on Microkernel Architecture

HormonyOS adopts a new micro-core design, which has stronger security features and low latency. The basic idea of microkernel design is to simplify the functions of the kernel, to realize as many system services as possible in user states outside the kernel, and to add security protection to each other. Microkernels only provide the most basic services, such as multi-process scheduling and multi-process communication.

![](assets/img/harmony/harmonyos3.jpg)

HormonyOS applies microkernel technology to Trusted Execution Environment (TEE), and reshapes trusted security through formal methods. Formal method is an effective means to verify the correctness of the system and the absence of loopholes from the source by using mathematical methods. Traditional verification methods such as function verification and simulation attack can only be validated in limited scenarios, while formal methods can validate all software running paths through data model. For the first time, Hongmeng OS has applied formal methods to terminal TEE, which significantly improves the security level. At the same time, because the code amount of Hongmeng OS micro-kernel is only one thousandth of that of Linux macro-kernel, its attack probability is greatly reduced.

* #### 4. Through unified IDE to support a single development, multi-terminal deployment, achieve cross-terminal ecological sharing

![](assets/img/harmony/harmonyos4.jpg)

HormonyOS relies on multi-terminal development IDE, multi-language unified compilation, distributed architecture Kit to provide screen layout control and interactive automatic adaptation, support control dragging, preview-oriented visual programming, so that developers can efficiently build multi-terminal automatic running App based on the same project, to achieve a real one-time development, multi-terminal. Deployment to achieve shared ecology across devices. Huawei Ark Compiler is the first static compiler to replace the Android virtual machine model, which allows developers to compile high-level languages into machine code at one time in the development environment. In addition, the Ark Compiler will support multi-language unified compilation in the future, which can greatly improve the development efficiency.


# IV . Resources


## 1. Official Resources

- [Official Site](https://developer.huawei.com)
- [Release Notices]
- [Security Notices]
- [Tutorials]
- [Documentation]

## 2. Download
- [Mirrors]

## 3. Architecture References
- [LiteOS](Huawei_LiteOS/README.md)
- [ABI] Application Binary Interface
- [EABI] Embedded Application Binary Interface

## 4. Hardware drivers
- [Device compatibility]
- [Standards & Protocols] 
- [Hareware Quality Specification]
- [Driver Development Kit]
- [Driver Samples]
- [Debugging Tools]
- [Security]
- [5th Gen Drivers and Firmware]
- [Boot and UEFI]
- [ACPI/SoC]
- [Wi-Fi]
- [USB]
- [Printer]


## 5. Developer Tools
#### Compiler
- [Ark] `Harmony`  is an Android-app compatible operating system, Ark compiler can improve Android system operation fluency by 24%, response speed by 44%, and the smoothness of the third-party application up to 60%.

#### SDK
[about SDK]

#### IDE
- [LiteOS IDE][Huawei LiteOS Studio](https://static.huaweicloud.com/upload/files/sdk/LiteOS_IDE.zip)

## 6. Community
- [Questions]

## 7. Books
[about books]

## 8. Products

- [Honor Smart Screen -- The first device to use the HoemonyOS](products/honor_smart_screen.md)

## 9. Videos

* [Instructional Videos]
* [Huawei HDC.2019 Harmony OS & EMUI10 Live Record](https://youtu.be/yUVGc7zpuKU)
* [Honor Smart Screen TV Official Trailer](https://youtu.be/GczF2CKIGPk)









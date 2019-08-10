## 概要说明

目前，社区有多种 Commit message 的写法规范。LiteOS采用的是Angular规范，这是目前使用最广的写法，比较合理和系统化，并且有配套的工具。


## Commit message的作用
格式化的Commit message有几个好处：

- 提供更多的历史信息，方便快速浏览
- 可以过滤某些commit（比如文档改动），便于快速查找信息。
- 可以直接从commit生成Change log。

## LiteOS Commit message的格式

每次提交，Commit message 都包括三个部分：Header，Body 和 Footer。

	<type>(<scope>): <subject>
	空一行
	<body>
	空一行
	<footer>
	比如：

    fix(stm32f411):fix stm32f411 migration guide file error

    fix some error in stm32f411re migration guide file.

    Close #75

### Header格式
 Header部分只有一行，包括三个字段：type（必需）、scope（可选）和subject（必需）。

- type

	type用于说明 commit 的类别，只允许使用下面7个标识。

	feat：新功能（feature）

	fix：修补bug

	docs：文档（documentation）

	style： 格式（不影响代码运行的变动）

	refactor：重构（即不是新增功能，也不是修改bug的代码变动）

	test：增加测试

	chore：构建过程或辅助工具的变动

	如果type为feat和fix，则该 commit 将肯定出现在 Change log 之中。其他情况（docs、chore、style、refactor、test）由你决定，要不要放入 Change log，建议是不要。

- scope

	scope用于说明 commit 影响的范围，比如LiteOS kernel的core修改影响全部则填写all，如果只修改stm32f411的则填写stm32f411。

- subject

	subject是 commit 目的的简短描述，不超过50个字符。

	以动词开头，使用第一人称现在时，比如change，而不是changed或changes

	第一个字母小写, 结尾不加句号（.）

### Body格式

Body 部分是对本次 commit 的详细描述，可以分成多行。下面是一个范例。

    Add porting contest board projects to LiteOS
    Board list:
    Arduino-M0-PRO
    ATSAM4S-XPRO
    ATSAMD21-XPRO
    EFM32-SLSTK3400A
    EFM32-SLSTK3401A
    EFM32-STK3700
    FRDM-KL26Z
    FRDM-KW41Z


有两个注意点。

- 使用第一人称现在时，比如使用change而不是changed或changes。

- 应该说明代码变动的动机，以及与以前行为的对比。


### Footer格式

Footer 部分只用于两种情况。

- 不兼容变动

如果当前代码与上一个版本不兼容，则 Footer 部分以BREAKING CHANGE开头，后面是对变动的描述、以及变动理由和迁移方法。

	BREAKING CHANGE: isolate scope bindings definition has changed.

    	To migrate the code follow the example below:

    	Before:

		scope: {
      		myAttr: 'attribute',
    	}

		After:

		scope: {
			myAttr: '@',
		}
		The removed `inject` wasn't generaly useful for directives so there should be no code using it.

- 关闭 Issue

	如果当前 commit 针对某个issue，那么可以在 Footer 部分关闭这个 issue 。

	Closes #16, #24, #92

## 更多参考

更详细的commit规则请参考原始的规范说明
 
[Angular规范](https://github.com/mychaser/docgather/blob/master/GitCommitMessageConventions.pdf)

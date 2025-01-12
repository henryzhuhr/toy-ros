
## python 魔法函数 __init__
- 概念：python中魔法函数是一种特殊的方法。它们以双下划线开头和结尾（例如__init__，__str__），在**类定义**中用于实现某些特殊行为或协议。魔法函数的使用可以使自定义对象表现得像内置对象，可以实现操作符重载、对象的表现形式转换等等。

  
&emsp;&emsp;`__init__` 是一个特殊方法，用于初始化对象。当创建一个新对象时，`__init__` 方法会自动调用。这个方法的参数是对象本身（self）和其他参数，用于初始化对象的属性。

## why `__name__ == "__main__"`
- `__name__` 是一个特殊变量，用于表示模块的名字。
- `__main__` 是 Python 中的一个特殊变量名，用于表示当前运行的模块。
- 当一个模块被直接运行时，`__name__` 的值为`__main__`；当一个模块被导入时，`__name__` 的值为模块的名字。因此，通过判断`__name__` 的值，可以确定模块是被导入还是被直接运行。


## 什么是进程，为什么说一个node就是一个进程. 
- 进程：进程是操作系统中资源分配的基本单位，每个进程都有自己的内存空间、数据段、代码段等资源。
- 线程：线程是进程中的一个执行单元，一个进程可以包含多个线程，线程共享进程的资源。
  
&emsp;&emsp;在操作系统中，进程是资源分配的基本单位，线程是操作系统调度的基本单位。一个进程可以包含多个线程，线程共享进程的资源，但是线程之间有自己的栈空间，可以独立运行。因此，**线程之间的通信比进程之间的通信更加方便快捷**。

&emsp;&emsp;通信方面:
- 进程之间的通信需要通过进程间通信（IPC）机制，如管道、消息队列、共享内存等
- 线程由操作系统或线程库调度，线程切换的开销较小

## 节点之间的通信为什么类似于可以理解为进程的通信
- 独立性上：在ros系统中，每个节点是一个独立的实体，具有自己的运行环境和资源。进程同样如此有着之间的独立的内存空间和资源。
- 通信机制上：节点之间的通信通过消息传递，服务调用的机制实现，类似于进程之间通过IPC机的通信，如消息队列、管道、共享内存等。
- 消息队列用来解决通信问题，上游（也可以是进程）发送一个消息后，无需过多等待直接进行下一项，**下游多个服务订阅到消息后各自执行功能**使用异步通信方式对各个模块间或者说进程间的调用进行解耦，这样可以提高系统的并发性，提高系统的性能。
  
## python decorator @property（装饰器）
&emsp;&emsp;1.装饰器的概念：装饰器是为了解耦代码的逻辑、提高代码的复用性，并且让函数和类的行为在不修改其原始代码的前提下得以扩展。能有效地将某些附加行为抽象出来，使得主业务逻辑（例如函数本身）保持简洁和专注。装饰器本质上是一个接受函数作为参数并返回一个新函数的函数。它常用于:

- 在函数执行前后添加额外的行为。
- 修改函数的输入或输出。
- 为函数增加一些元数据。

基本结构：
```python
def decorator(func):
    def wrapper():
 
        print("Before function call")
 
        func()  # 调用原函数
 
        print("After function call")
 
    return wrapper
 
# 使用装饰器
@decorator
def say_hello():
    print("Hello!")
 
# 调用装饰后的函数
say_hello()
```

输出结果为：
```python
Before function call

Hello!

After function call
```

@decorator 语法实际上是`say_hello = decorator(say_hello)`，即用装饰器修改 say_hello 函数。


&emsp;&emsp;python有一些内置装饰器,其中`@property` 装饰器用于**将方法变为属性**，这意味着该方法的调用方式就像访问属性一样，而不需要显式调用方法。常用于定义只读属性，或者为属性添加 getter 和 setter 方法。

## 什么是属性，什么是方法
- 属性是类的实例变量，用于存储对象的状态或数据。属性通常在类的构造函数（__init__ 方法）中定义，并且可以通过对象访问和修改。
```python
class Person:
    def __init__(self, name, age):
        self.name = name  # name 是一个属性
        self.age = age    # age 是一个属性

# 创建一个 Person 对象
p = Person("Alice", 30)

# 访问属性
print(p.name)  # 输出: Alice
print(p.age)   # 输出: 30

# 修改属性
p.age = 31
print(p.age)   # 输出: 31
```
- 方法是类的函数，用于定义对象的行为。方法通常在类中定义，并且可以通过对象调用。方法可以访问和修改对象的属性。
```python
class Person:
    def __init__(self, name, age):
        self.name = name
        self.age = age

    def greet(self):
        print(f"Hello, my name is {self.name} and I am {self.age} years old.")  # greet 是一个方法

# 创建一个 Person 对象
p = Person("Alice", 30)

# 调用方法
p.greet()  # 输出: Hello, my name is Alice and I am 30 years old.

```


## difference way to format string in python (f-string, format, %)

- 1.&emsp;f-string方法(使用起来非常简洁和直观)
```python
name = "Alice"
age = 30
formatted_string = f"Hello, my name is {name} and I am {age} years old."
print(formatted_string)  # 输出: Hello, my name is Alice and I am 30 years old.
```
- 2.&emsp;str.format() 方法
```python
name = "Alice"
age = 30
formatted_string = "Hello, my name is {} and I am {} years old.".format(name, age)
print(formatted_string)  # 输出: Hello, my name is Alice and I am 30 years old.
```
- 3.&emsp;百分号 (%) 格式化(类似于c语言)
```python
name = "Alice"
age = 30
formatted_string = "Hello, my name is %s and I am %d years old." % (name, age)
print(formatted_string)  # 输出: Hello, my name is Alice and I am 30 years old.
```
- 4.&emsp;使用 str() 函数
&emsp;&emsp;

如果只是简单地将变量转换为字符串，可以使用 str() 函数。
```python
num = 123
formatted_string = "The number is " + str(num)
print(formatted_string)  # 输出: The number is 123
```
- 5.&emsp;使用 repr() 函数,类似于str()函数
```python
num = 123
formatted_string = "The number is " + repr(num)
print(formatted_string)  # 输出: The number is 123
```
  
  ## type comment in python (such as `a: int = 1`)
  
  ```python
  num: str = 1
  print(num, type(num))
  ```
  &emsp;&emsp;可以这样的原因是，python，支持变量类型注解用于指示变量的预期类型但并不会强制改变变量类型，Python仍然是动态类型语言，变量的类型可以在运行时改变。这样可以提高代码的可读性，方便代码的维护和调试。

  - 类型注释主要用于静态类型检查工具，如 mypy。这些工具可以在代码运行前检查类型一致性，帮助发现潜在的类型错误。
  - 例如，mypy 会警告 num 被注释为 str 类型，但赋值为 int 类型。

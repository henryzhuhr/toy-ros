import math

def main():
    ## python的print()函数
    print("妈妈"+" 爸爸")  # 字符串拼接
    print("我是第一行\n我是第二行")  # \n表示换行
    print("""大汉之剑天上来
          奔流到海不复回
          皇后""")  # 三引号可以输入多行文本
    print(math.sin(1))
    s = "hello"
    print(len(s))  # 对字符串求长度

#------------------------------------------------
    ## python的数据类型
    b1 = True
    b2 = False  # 布尔类型
    n = None  # 空值类型
    print(type(b1))
    print(type(n))  # type()函数可以返回变量的类型
    
#------------------------------------------------
    ## python的input()函数
    name = input("请输入你的姓名：")
    print("你好" + name)  # 问答互动程序
    
#------------------------------------------------
    ## python条件语句
    mood_index=int(input("今晚女朋友心情指数"))  # input()函数返回的数据类型是str，不能直接和整数比较，必须先把str转换成整数
    if mood_index>=90:
        print("今晚女朋友心情好")
    elif mood_index>=60:
        print("今晚女朋友心情一般")
    else:
        print("今晚女朋友心情不好")  # if elif else语句
        
    if mood_index>=90:
        print("今晚女朋友心情好")
        if mood_index==100:
            print("今晚女朋友心情极好")
        else:
            print("今晚女朋友依然好")
    else :
        print("今晚女朋友心情不好")  # 嵌套if语句，多条件判断
        
 #------------------------------------------------
    house_work_count = int(input("请输入你做家务的次数："))
    red_envelope_count = int(input("请输入你收到红包的次数："))
    shopping_count = int(input("请输入你购物的次数："))
    has_been_angry = input("你是否生气了？(yes/no)：").lower() == 'yes'
    
    if (house_work_count > 10 and red_envelope_count > 1 and shopping_count > 4 and not 
        has_been_angry):
        print("摩拳擦掌等待Switch")
    else:
        print("Switch随风而去...")   # python逻辑运算符
    # not and or,优先级为not>and>or

#------------------------------------------------
    ## python列表
    #列表可改变不用重新赋值，但其他int类数据类型不能直接改变，得重新赋值
    #列表可以放不同类型的数据
    list1 = [1,2,3,4,5]   # 用中括号创建一个列表
    list1.append(6)  # 添加元素
    list1.remove(3)  # 删除元素
    len(list1)  # 求列表长度
    list1[0]  # 取索引第一个元素，且可以直接对其赋值
    max(list1)  # 求列表最大值
    min(list1)  # 求列表最小值
    sorted(list1)  # 返回排序好的新列表但不改变原列表
    
    
    
    
    ##元组不能改变，只能重新赋值
    tuple1 = (1,2,3,4,5)  # 用小括号
    tuple1[0]  # 取索引第一个元素
    
    ##python字典
    dict1 = {"name":"Tom","age":"20,"}  # 用大括号
    dict1["电话号码"]="123456789"  # 添加元素
    print("name" in dict1)  # 判断是否存在   in用于返回布尔值Ture or False、
    del dict1["name"]  # 删除元素
    len(dict1)  # 求字典长度
    
    
    #for循环{
        # 。。。
    #}
    
    
    
    
    """Host ubuntu
       Hostname henryapp.online
       User zmy
       port 6000
       ForwardAgent yes
       PreferredAuthentications publickey
       IdentityFile ~/.ssh/sshkey-ubuntu"""
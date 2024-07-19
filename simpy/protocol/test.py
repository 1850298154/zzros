
from base_enum import Base_Enum

# 定义一个枚举类
class MyEnum(Base_Enum):
    FIRST = "apple"
    SECOND = "banana"
    THIRD = "orange"


# 使用枚举成员
print(MyEnum.FIRST.value)  # 输出: apple
print(MyEnum.SECOND.value)  # 输出: banana
print(MyEnum.THIRD.value)  # 输出: orange

# 测试静态方法
fruit = "banana"
enum_member = MyEnum.get_enum_member_from_value(fruit, MyEnum)
print(enum_member)  # 输出: MyEnum.SECOND
print(MyEnum(fruit))  # 输出: MyEnum.SECOND
print(MyEnum(MyEnum.FIRST))  # 输出: MyEnum.SECOND

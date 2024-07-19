
from enum import Enum




class Base_Enum(Enum):

    @staticmethod
    def get_enum_member_from_value(value, MyEnum_cls):
        """根据枚举成员的值来获取对应的枚举成员
        """
        for member in MyEnum_cls:
            if member.value == value:
                return member
        raise ValueError("No enum member with value {}".format(value))

    @staticmethod
    def get_all_members(MyEnum_cls):
        """获取所有枚举成员的列表
        """
        return list(MyEnum_cls)

    @staticmethod
    def has_member(member_name, MyEnum_cls):
        """检查枚举成员是否存在
        """
        return any(member_name == member.name for member in MyEnum_cls)

    @staticmethod
    def get_enum_member_from_name(member_name, MyEnum_cls):
        """根据枚举名称获取枚举成员
        """
        for member in MyEnum_cls:
            if member.name == member_name:
                return member
        raise ValueError("No enum member with name {}".format(member_name))

    @staticmethod
    def get_all_values(MyEnum_cls):
        """获取枚举成员的值列表
        """
        return [member.value for member in MyEnum_cls]

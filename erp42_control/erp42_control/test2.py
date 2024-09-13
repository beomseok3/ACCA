
import os
import sys

def set_param(node_name,param_name,value):
    try:
        a=os.system(f"ros2 param set /{node_name} {param_name} {value}")
        print(a)
    except Exception as e:
        print("d",e)
        
b =set_param("test","tes",10)

# 함수 잘 만들고 terminal log 문자열 succsess확인하든가 -->  state machine 적용하고 처리확인
import rclpy
from  rclpy.node import Node 




class PersonNode(Node):  # 继承自 Node 类
    def __init__(self, node_name:str,  name:str, age:int) -> None:
        super().__init__(node_name)
        self.get_logger().info("PersonNode 构造函数被调用")
        self.name = name
        self.age = age
        
    def eat(self, food_name:str):
        self.get_logger().info(f"{self.age} 岁的 {self.name} 正在吃 {food_name}")
        
def main():
    rclpy.init()
    node = PersonNode("person_node", "小明", 10)
    node.eat("苹果")
    rclpy.spin(node)    
    rclpy.shutdown()

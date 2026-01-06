from demo_python_pkg.person_node import PersonNode

class WriterNode(PersonNode):

    def __init__(self, name: str, age: int, book: str) -> None:
        print("WriterNode 构造函数被调用")
        super().__init__(name, age)
        self.book = book


def main():
    writer = WriterNode("小明", 11, "Python编程从入门到放弃")
    writer.eat("香蕉")
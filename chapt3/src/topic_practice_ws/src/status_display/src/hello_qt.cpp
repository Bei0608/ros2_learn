# include <QApplication>
# include <QLabel>
# include <QString>

int main(int argc, char *argv[])
{
  QApplication app(argc, argv);

  QLabel* label = new QLabel();
  QString message = QString::fromStdString("Hello, ROS 2 with Qt!");
  label->setText(message);
  label->show();
  app.exec();

  return 0;
}
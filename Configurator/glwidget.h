#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QOpenGLFunctions>
#include <QQuaternion>
#include <QVector2D>

QT_FORWARD_DECLARE_CLASS(QGLShaderProgram);

class GLWidget : public QGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit GLWidget(QWidget *parent = 0, QGLWidget *shareWidget = 0);
    ~GLWidget();

    void rotateBy(QQuaternion *q);

signals:
    void clicked();

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

private:
    void makeObject();

    QVector2D mouseLastPos;
    QVector3D rotationAxis;
    qreal angularSpeed;
    QQuaternion rotation;

    GLuint textures[6];
    QVector<QVector3D> vertices;
    QVector<QVector2D> texCoords;
    QGLShaderProgram *program;
};

#endif // GLWIDGET_H

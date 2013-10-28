#ifndef ASTARWIN_H
#define ASTARWIN_H

#include <QMainWindow>

#include <cmath>
#include <QList>
#include <QTimer>

namespace Ui {
    class AStarWin;
}

class AStarWin : public QMainWindow
{
        Q_OBJECT

    public:
        explicit AStarWin(QWidget *parent = 0);
        ~AStarWin();

    public slots:
        void updateSearch();

    private:
        Ui::AStarWin *ui;

        struct Vec2d {
            double x, y;

            Vec2d(double angle): x(std::cos(angle)), y(std::sin(angle)) {}
            Vec2d(): x(0.0), y(0.0) {}
            Vec2d(double _x, double _y): x(_x), y(_y) {}

            double angle() const {
                return std::atan2(y, x);
            }

            Vec2d operator*(double s) const {
                return Vec2d(x * s, y * s);
            }

            Vec2d operator/(double s) const {
                return Vec2d(x / s, y / s);
            }

            Vec2d operator+(const Vec2d& v) const {
                return Vec2d(x + v.x, y + v.y);
            }

            Vec2d operator-(const Vec2d& v) const {
                return Vec2d(x - v.x, y - v.y);
            }

            double dot(const Vec2d& v) const {
                return x * v.x + y * v.y;
            }

            double distanceSquared(const Vec2d& v) const {
                return (v.x - x) * (v.x - x) + (v.y - y) * (v.y - y);
            }

            double distance(const Vec2d& v) const {
                return std::sqrt(distanceSquared(v));
            }
        };

        struct Obstacle: public Vec2d {
            Vec2d pos;
            double radius;

            Obstacle(const Vec2d& p, double r): Vec2d(p), radius(r) {}
        };

        struct Pose: public Vec2d {
            Vec2d dir;

            Pose(): Vec2d(), dir() {}
            Pose(const Vec2d& p, double d): Vec2d(p), dir(d) {}

        };

        struct PathNode: public Pose {
            PathNode* parent;
            QList< PathNode* > children;
            double length;
            double sumCurve;

            PathNode(const Pose&p, double curve, PathNode* _parent = 0): Pose(p), parent(_parent), length(0.0), sumCurve(curve) {
                if(parent) {
                    length = parent->length + 1;
                    parent->children.append(this);
                }
            }

            ~PathNode() {
                qDeleteAll(children);
            }

            PathNode* turn(double angle) {
                return new PathNode(Pose(*this + dir, dir.angle() + angle), sumCurve + std::fabs(angle), this);
            }
        };

        typedef std::pair< double, PathNode* > ScoredPath;

        Pose source, target;
        QList< Obstacle > obstacles;
        QList< ScoredPath > paths;
        PathNode* root;

        QPoint vec2point(const Vec2d& v) const;

        double score(const PathNode* n) const;

        QTimer *timer;
        // QMainWindow interface
    public:
        QMenu *createPopupMenu();

        // QWidget interface
    protected:
        void paintEvent(QPaintEvent *);
        void mousePressEvent(QMouseEvent *);
        void mouseReleaseEvent(QMouseEvent *);
        void mouseDoubleClickEvent(QMouseEvent *);
        void mouseMoveEvent(QMouseEvent *);

        void drawPose(QPainter &painter, const Pose& p, const QColor &c);
};

#endif // ASTARWIN_H

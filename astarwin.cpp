#include "astarwin.h"
#include "ui_astarwin.h"

#include <QPainter>

#define WORLDX  30

#define MIN_DIST 0.05

AStarWin::AStarWin(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::AStarWin),
    root(0)
{
    ui->setupUi(this);

    source = Pose(Vec2d( -10, 10), M_PI/2);

    target = Pose(Vec2d( 10, -10), -M_PI/4);

    obstacles.append(Obstacle(Vec2d(0, 0), 7.0));
    obstacles.append(Obstacle(Vec2d(13, -9), 2.95));
    obstacles.append(Obstacle(Vec2d(9, -13), 2.95));

    paths.append(ScoredPath(0, root = new PathNode(source, 0)));

    timer = new QTimer(this);
    timer->setInterval(0);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateSearch()));
    timer->start();
}

AStarWin::~AStarWin()
{
    delete ui;
    delete root;
}

void AStarWin::updateSearch()
{
    ScoredPath sp = paths.first();

    if(sp.second->distance(target) < 0.2 && std::acos(sp.second->dir.dot(target.dir)) < M_PI/32)
    {
        timer->stop();
        return;
    }

    paths.pop_front();

    PathNode* n[3] = {
        sp.second->turn(-M_PI/16),
        sp.second->turn(0),
        sp.second->turn(M_PI/16)
    };

    bool valid[3] = {true, true, true};

    for(int i = 0; i < 3; ++i) {
        foreach(const ScoredPath& sn, paths) {
            if((n[i]->dir + *n[i]).distanceSquared(sn.second->dir + *sn.second) < MIN_DIST * MIN_DIST)
                valid[i] = false;
        }

        if(valid[i])
            foreach(const Obstacle& o, obstacles)
                if(n[i]->distanceSquared(o) < o.radius * o.radius)
                    valid[i] = false;

        if(valid[i]) paths.append(ScoredPath(score(n[i]), n[i]));
    }
    std::sort(paths.begin(), paths.end());
    update();
}

QPoint AStarWin::vec2point(const AStarWin::Vec2d &v) const
{
    const int w = width(), h = height(), WORLDY = (WORLDX * h / w);
    const int x = w/2.0 + v.x / WORLDX * w/2.0;
    const int y = h/2.0 - v.y / WORLDY * h/2.0;

    return QPoint(x, y);
}

double AStarWin::score(const AStarWin::PathNode* n) const
{
    return n->length +
            n->distance(target);// + n->sumCurve / (M_PI);
}


QMenu *AStarWin::createPopupMenu()
{
    return 0;
}

void AStarWin::paintEvent(QPaintEvent *)
{
    const int w = width(), h = height(), WORLDY = (WORLDX * h / w);

    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);

    foreach(const Obstacle& o, obstacles) {
        int rx = o.radius / WORLDX * w/2.0;
        int ry = o.radius / WORLDY * h/2.0;
        painter.setPen(Qt::red);
        painter.drawEllipse(vec2point(o), rx, ry);
    }

    drawPose(painter, source, Qt::blue);
    drawPose(painter, target, Qt::green);

    foreach(const ScoredPath& sp, paths) {
        drawPose(painter, *sp.second, Qt::cyan);
    }

    ScoredPath sp = paths.first();
    PathNode *current = sp.second;
    while(current) {
        drawPose(painter, *current, Qt::magenta);
        current = current->parent;
    }
}

void AStarWin::mousePressEvent(QMouseEvent *)
{
}

void AStarWin::mouseReleaseEvent(QMouseEvent *)
{
}

void AStarWin::mouseDoubleClickEvent(QMouseEvent *)
{
}

void AStarWin::mouseMoveEvent(QMouseEvent *)
{
}

void AStarWin::drawPose(QPainter& painter, const AStarWin::Pose &p, const QColor& c)
{
    QPoint l = vec2point(p);
    QPoint e = vec2point(p.dir + p);
    painter.setPen(c);
    painter.drawLine(l, e);
}

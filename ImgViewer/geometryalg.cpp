#include "geometryalg.h"

#include <algorithm>

//判断点在线段上
bool IsPointOnLine(double px0, double py0, double px1, double py1, double px2, double py2)
{
    bool flag = false;
    double d1 = (px1 - px0) * (py2 - py0) - (px2 - px0) * (py1 - py0);
    if ((abs(d1) < EPSILON) && ((px0 - px1) * (px0 - px2) <= 0) && ((py0 - py1) * (py0 - py2) <= 0))
    {
        flag = true;
    }
    return flag;
}

//判断两线段相交
bool IsIntersect(double px1, double py1, double px2, double py2, double px3, double py3, double px4, double py4)
{
    bool flag = false;
    double d = (px2 - px1) * (py4 - py3) - (py2 - py1) * (px4 - px3);
    if (d != 0)
    {
        double r = ((py1 - py3) * (px4 - px3) - (px1 - px3) * (py4 - py3)) / d;
        double s = ((py1 - py3) * (px2 - px1) - (px1 - px3) * (py2 - py1)) / d;
        if ((r >= 0) && (r <= 1) && (s >= 0) && (s <= 1))
        {
            flag = true;
        }
    }
    return flag;
}

//判断点在多边形内
bool Point_In_Polygon_2D(double x, double y, const std::vector<Vector2d> &POL)
{
    bool isInside = false;
    int count = 0;

    //
    double minX = DBL_MAX;
    for (int i = 0; i < POL.size(); i++)
    {
        minX = std::min(minX, POL[i].x);
    }

    //
    double px = x;
    double py = y;
    double linePoint1x = x;
    double linePoint1y = y;
    double linePoint2x = minX -10;			//取最小的X值还小的值作为射线的终点
    double linePoint2y = y;

    //遍历每一条边
    for (int i = 0; i < POL.size() - 1; i++)
    {
        double cx1 = POL[i].x;
        double cy1 = POL[i].y;
        double cx2 = POL[i + 1].x;
        double cy2 = POL[i + 1].y;

        if (IsPointOnLine(px, py, cx1, cy1, cx2, cy2))
        {
            return true;
        }

        if (fabs(cy2 - cy1) < EPSILON)   //平行则不相交
        {
            continue;
        }

        if (IsPointOnLine(cx1, cy1, linePoint1x, linePoint1y, linePoint2x, linePoint2y))
        {
            if (cy1 > cy2)			//只保证上端点+1
            {
                count++;
            }
        }
        else if (IsPointOnLine(cx2, cy2, linePoint1x, linePoint1y, linePoint2x, linePoint2y))
        {
            if (cy2 > cy1)			//只保证上端点+1
            {
                count++;
            }
        }
        else if (IsIntersect(cx1, cy1, cx2, cy2, linePoint1x, linePoint1y, linePoint2x, linePoint2y))   //已经排除平行的情况
        {
            count++;
        }
    }

    if (count % 2 == 1)
    {
        isInside = true;
    }

    return isInside;
}

// v1 = Cross(AB, AC)
// v2 = Cross(AB, AP)
// 判断矢量v1和v2是否同向
bool SameSide(Vector3d A, Vector3d B, Vector3d C, Vector3d P)
{
    Vector3d AB = B - A;
    Vector3d AC = C - A;
    Vector3d AP = P - A;

    Vector3d v1 = AB.Cross(AC);
    Vector3d v2 = AB.Cross(AP);

    // v1 and v2 should point to the same direction
    return v1.Dot(v2) >= 0 ;
    //return v1.Dot(v2) > 0;
}

// 判断点P是否在三角形ABC内(同向法)
bool PointinTriangle(Vector3d A, Vector3d B, Vector3d C, Vector3d P)
{
    return SameSide(A, B, C, P) && SameSide(B, C, A, P) && SameSide(C, A, B, P);
}

bool Point_In_Triangle_2D(const Vector2d & A, const Vector2d & B, const Vector2d & C, const Vector2d & P)
{
    return  (B.x - A.x) * (P.y - A.y) > (B.y - A.y) * (P.x - A.x) &&
            (C.x - B.x) * (P.y - B.y) > (C.y - B.y) * (P.x - B.x) &&
            (A.x - C.x) * (P.y - C.y) > (A.y - C.y) * (P.x - C.x) ? false : true;
}


//计算三点成面的法向量
void Cal_Normal_3D(const Vector3d& v1, const Vector3d& v2, const Vector3d& v3, Vector3d &vn)
{
    //v1(n1,n2,n3);
    //平面方程: na * (x – n1) + nb * (y – n2) + nc * (z – n3) = 0 ;
    double na = (v2.y - v1.y)*(v3.z - v1.z) - (v2.z - v1.z)*(v3.y - v1.y);
    double nb = (v2.z - v1.z)*(v3.x - v1.x) - (v2.x - v1.x)*(v3.z - v1.z);
    double nc = (v2.x - v1.x)*(v3.y - v1.y) - (v2.y - v1.y)*(v3.x - v1.x);

    //平面法向量
    vn.Set(na, nb, nc);
}

//已知空间三点组成的面求该面上某点的Z值
void CalPlanePointZ(const Vector3d& v1, const Vector3d& v2, const Vector3d& v3, Vector3d& vp)
{
    Vector3d vn;
    Cal_Normal_3D(v1, v2, v3, vn);

    if (vn.z != 0)				//如果平面平行Z轴
    {
        vp.z = v1.z - (vn.x * (vp.x - v1.x) + vn.y * (vp.y - v1.y)) / vn.z;			//点法式求解
    }
}

GeometryAlg::GeometryAlg()
{

}

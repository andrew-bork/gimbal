#ifndef MATH_INCLUDE_GUARD
#define MATH_INCLUDE_GUARD

/**
    Pulled from an older project https://github.com/andrew-bork/autonomous/blob/main/include/math/math.h 
    A set of complicated math objects.
*/
namespace math {

        /**
         * @brief 3D vector.
         * 
         */
        struct vector{
                double x,y,z;

                /**
                 * @brief Construct a zero vector.
                 * 
                 */
                vector();
                /**
                 * @brief Construct a new vector: [x, 0, 0]
                 * 
                 * @param x 
                 */
                vector(double x);
                /**
                 * @brief Construct a new vector: [x, y, 0]
                 * 
                 * @param x 
                 * @param y 
                 */
                vector(double x, double y);
                /**
                 * @brief Construct a new vector: [x, y, z]
                 * 
                 * @param x 
                 * @param y 
                 * @param z 
                 */
                vector(double x, double y, double z);

                /**
                 * @brief Vector addition.
                 * 
                 * @param r 
                 * @return vector 
                 */
                vector operator+ (const vector& r);
                /**
                 * @brief Vector scalar scaling.
                 * 
                 * @param s 
                 * @return vector 
                 */
                vector operator* (const double& s);
        };

        /**
         * @brief QUARETNERIOSN. 
         * Basically a better way to represent rotations.
         * https://en.wikipedia.org/wiki/Quaternion (Have fun)
         *  Q2 * Q1 -> rotation Q1 is applied before rotation Q2.
         */
        struct quarternion{
                double w, x, y, z;
                bool unit; // Unit quarternion flag.

                /**
                 * @brief Construct a zero quarternion
                 * 
                 */
                quarternion();
                /**
                 * @brief Construct a new quarternion object (x, y, z)
                 * 
                 * @param x 
                 * @param y 
                 * @param z 
                 */
                quarternion(double x, double y, double z);
                /**
                 * @brief Construct a new quarternion object (w, x, y, z)
                 * 
                 * @param w 
                 * @param x 
                 * @param y 
                 * @param z 
                 */
                quarternion(double w, double x, double y, double z);
                /**
                 * @brief Construct a new quarternion object (w, x, y, z, unit)
                 * 
                 * @param w 
                 * @param x 
                 * @param y 
                 * @param z 
                 * @param unit 
                 */
                quarternion(double w, double x, double y, double z, bool unit);

                /**
                 * @brief Element-wise addition.
                 * 
                 * @param r 
                 * @return quarternion 
                 */
                quarternion operator+ (const quarternion& r);
                /**
                 * @brief Quarternion multiplicatoin. "Rotation"
                 * Look at the wikipedia article.
                 * r will get applied before this.
                 * 
                 * @param r this rotation will get applied first
                 * @return quarternion 
                 */
                quarternion operator* (const quarternion& r);

                /**
                 * @brief Invert the quarternion. Check wikipedia
                 * 
                 * @param n 
                 * @return quarternion 
                 */
                static quarternion inverse(const quarternion& n);
                /**
                 * @brief Get the quareternion conjugate. Used for rotations. Check Wikipedia.
                 * 
                 * @param n 
                 * @return quarternion 
                 */
                static quarternion conjugate(const quarternion& n);
                /**
                 * @brief Get a new quarternion that represents the rotation around an axis by an angle theta.
                 * 
                 * @param theta 
                 * @param axis 
                 * @return quarternion 
                 */
                static quarternion rotateAxis(double theta, const vector& axis);
                /**
                 * @brief Get a new quarternion that represents euler angles.
                 * x - roll
                 * y - pitch
                 * z - yaw
                 * The rotations are applied in the order Z-Y-X
                 * 
                 * @param euler 
                 * @return quarternion 
                 */
                static quarternion fromEulerZYX(const vector& euler);
                
                /**
                 * @brief Convert a quarternion back into euler angles
                 * x - roll
                 * y - pitch
                 * z - yaw
                 * The rotations are applied in the order Z-Y-X
                 * 
                 * @param q 
                 * @return vector 
                 */
                static vector toEuler(const quarternion& q);
                /**
                 * @brief Convert a quarternion into a vector. The vector's direction is the rotation axis and the vector's magnitude is the rotation angle.
                 * 
                 * @param q 
                 * @return vector 
                 */
                static vector toMagAxis(const quarternion& q);

                /**
                 * @brief Rotate a vector using a quarternion.
                 * v' = p * in * p'
                 * Look at the wikipedia for more details.
                 * 
                 * @param q 
                 * @param in 
                 * @return vector 
                 */
                static vector rotateVector(math::quarternion& q, math::vector& in);
        };
        
        /**
         * @brief Magnitude of a quarternion.
         * 
         * @param q 
         * @return double 
         */
        double length(const math::quarternion& q);
        /**
         * @brief Magnitude (length) of a vector.
         * 
         * @param v 
         * @return double 
         */
        double length(const math::vector& v);
}






/**
 * IMPLEMENTATIONS OF MATH STUFF.
 * 
 */

math::quarternion::quarternion(){
    w = x = y = z = 0;
    unit = false;
}

math::quarternion::quarternion(double x, double y, double z){
    w = 0;
    this->x = x;
    this->y = y;
    this->z = z;
    unit = false;
}
math::quarternion::quarternion(double w, double x, double y, double z){
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
    unit = false;
}

math::quarternion::quarternion(double w, double x, double y, double z, bool unit){
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
    this->unit = unit;
}



math::quarternion math::quarternion::operator*(const math::quarternion& r){
    quarternion res;
    res.w = w*r.w - x*r.x - y*r.y - z*r.z;
    res.x = r.w*x + r.x*w + r.y*z - r.z*y;
    res.y = r.w*y + r.y*w + r.z*x - r.x*z;
    res.z = r.w*z + r.z*w + r.x*y - r.y*x;
    return res;
}

math::quarternion math::quarternion::operator+(const math::quarternion& r){
    quarternion res;
    res.w = r.w + w;
    res.x = r.x + x;
    res.y = r.y + y;
    res.z = r.z + z;
    return res;
}

math::quarternion math::quarternion::conjugate(const math::quarternion& q){
    quarternion res;
    res.w = q.w;
    res.x = -q.x;
    res.y = -q.y;
    res.z = -q.z;
    return res;
}

double math::length(const math::quarternion& q){
    return sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
}

double math::length(const math::vector& v){
    return sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
}

math::quarternion math::quarternion::inverse(const math::quarternion& q){
    double len;
    if(q.unit) len = 1;
    else len = length(q);
    quarternion res;
    res.w = q.w;
    res.x = -q.x;
    res.y = -q.y;
    res.z = -q.z;
    return res;
}
math::quarternion math::quarternion::rotateAxis(double theta, const math::vector& axis){
    quarternion res;
    res.w = cos(theta/2);
    res.x = axis.x*sin(theta/2);
    res.y = axis.y*sin(theta/2);
    res.z = axis.z*sin(theta/2);
    return res;
}

math::quarternion math::quarternion::fromEulerZYX(const math::vector& euler){
    double cy = cos(euler.z*0.5);
    double sy = sin(euler.z*0.5);
    double cp = cos(euler.y*0.5);
    double sp = sin(euler.y*0.5);
    double cr = cos(euler.x*0.5);
    double sr = sin(euler.x*0.5);

    quarternion res;
    res.w = cr*cp*cy + sr*sp*sy;
    res.x = sr*cp*cy - cr*sp*sy;
    res.y = cr*sp*cy + sr*cp*sy;
    res.z = cr*cp*sy - sr*sp*cy;
    return res;
}

math::vector math::quarternion::toEuler(const math::quarternion& q){
    double  sin_p = 2*(q.w*q.y - q.z*q.x);
    if(sin_p >=1){
        vector res(-2*atan2(q.x, q.w), 1.570796326794897, 0);
        return res;
    }else if(sin_p <= -1){
        vector res(2 *atan2(q.x, q.w), -1.570796326794897,  0);
        return res;
    }
    //asin(2*(q.w*q.y - q.z*q.x));
    vector res(atan2(2*(q.w*q.x+q.y*q.z), 1 - 2* (q.x*q.x + q.y*q.y)), asin(sin_p), atan2(2*(q.w*q.z+q.x*q.y), 1 - 2 * (q.y*q.y+q.z*q.z)));
    return res;
}

math::vector math::quarternion::toMagAxis(const math::quarternion& q){
    double s = 1 - q.w*q.w;
    double mag = acos(q.w)*2;

    if(q.w > -0.00001 && q.w < 0.00001){
            vector res(0,0,0);
            return res;
    }
    vector res(q.x*mag/s, q.y*mag/s,q.z*mag/s);
    return res;
}

math::vector::vector (){
    x = y = z = 0;
}
math::vector::vector (double _x, double _y){
    x = _x;
    y = _y;
    z = 0;
}
math::vector::vector (double _x, double _y, double _z){
    x = _x;
    y = _y;
    z = _z;
}

math::vector math::vector::operator+ (const math::vector& r){
    vector res(x+r.x,y+r.y,z+r.z);
    return res;
}

math::vector math::vector::operator*(const double& s){
    vector res(x*s, y*s, z*s);
    return res;
}

math::vector math::quarternion::rotateVector(math::quarternion& q, math::vector& r){
    quarternion temp(0, r.x, r.y, r.z);
    temp = q * temp * quarternion::conjugate(q);
    vector out = math::vector(temp.x, temp.y, temp.z);
    return out;
}


#endif
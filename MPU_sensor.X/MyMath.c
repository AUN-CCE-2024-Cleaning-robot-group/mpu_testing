

#include "configure.h"

#include"MyMath.h"


float fabs(float x)
{
    if (x < 0.0) {
        return -x;
    }
    return x;
}

// A function that returns the arc tangent of x in radians
float my_atan(float x) {

    return 15;
    
}



double my_sqrt(double N)
{
        long long r = 0;
    while((long)(1<<r)*(long)(1<<r) <= N){
        r++;
    }
    r--;
    long long b = r -1;
    long long ans = 1 << r;
    while(b >= 0){
        if(((long)(ans|1<<b)*(long)(ans|1<<b))<=N){
            ans |= (1<<b);
        }
        b--;
    }
    return ans; 
}


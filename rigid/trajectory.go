package rigid

import (
  "math"
)

type Path struct {
  Joints  [][]float64 
  //Time    []float64    // when Time != nil use it for steps
  //Tmin    float64
}

// Linear interpolation
// 0 <= s <= 1
func (p Path) GetLinear(s float64, res []float64) bool {
  n := len(p.Joints)
  if  n == 0 || s < 0 || s > 1 {
    return false
  } else if n == 1 {
    copy(res, p.Joints[0])
    return true 
  }
  n1 := float64(n - 1)
  k := n1*s 
  ik := math.Floor(k)      // int part
  nk := int(ik)
  copy(res, p.Joints[nk])
  k -= ik                 // float part 
  if k > 0 {
    k /= n1 
    for i := 0; i < len(res); i++ {
      res[i] += (p.Joints[nk+1][i]-res[i])*k   // res[i] == p.Joints[nk][i] 
    }
  }
  return true 
}

type Polynomial []float64 

// Find value 
func (p Polynomial) Val(x float64) float64 {
  res := p[0]
  for i := 1; i < len(p); i++ {
    res = res * x + p[i] 
  }
  return res 
}

// Derivative
func (p Polynomial) Der() Polynomial {
  var res Polynomial 
  if len(p) > 1 {
    n := len(p)-1
    res = make([]float64,n) 
    for i := 0; i < n; i++ {
      res[i] = float64(n-i) * p[i]
    }
  } else {
    res = Polynomial{0}
  }
  return res 
}

// Evaluate d/dx value
func (p Polynomial) Val1d (x float64) float64 {
  var res float64 
  n := len(p) - 1
  if n > 0 {
    res = float64(n)*p[0]
    for i := 1; i < n; i++ {
      res = res * x + float64(n-i) * p[i] 
    }
  } else {
    res = 0
  }
  return res
}

// Evaluate d2/dx2 value 
func (p Polynomial) Val2d (x float64) float64 {
  var res float64
  n := len(p) - 1
  m := n - 1
  if m > 0 {
    res = float64(n*m)*p[0] 
    for i := 1; i < m; i++ {
      res = res * x + float64((n-i)*(m-i)) * p[i]
    }
  } else {
    res = 0
  }
  return res 
}


/*
func TrapezProfile(d, vmax, amax float64) *Path {
  var p Path
  ta := vmax / amax
  tb := d / vmax 
  del := 1e-6
  
  if tb < ta {
    // triangle
    ta = math.Sqrt(d / amax)
    tb = ta
    vmax = ta * amax 
    p.Joints = [][]float64 {
                 []float64 {0,    0.5*(1-del),  0.5,  0.5*(1+del), 1},   // position
                 []float64 {0,   vmax*(1-del), vmax, vmax*(1-del), 0},   // velocity
                 []float64 {amax,        amax,    0,    -amax, -amax},   // acceleration
               }
    p.Time = []float64 {0, 0.5-del, 0.5, 0.5+del, 1}
  } else {
    // trapez
    t1 := ta / (ta + tb)
    t2 := tb / (ta + tb)
    p.Joints = [][]float64 {
                 []float64 {0,   t1*(1-del),   t1,   t2,   t2*(1+del), 1},  // position
                 []float64 {0, vmax*(1-del), vmax, vmax, vmax*(1-del), 0},  // velocity
                 []float64 {amax,      amax,    0,    0,    -amax, -amax},  // acceleration
               }
    
    
  }
}
*/
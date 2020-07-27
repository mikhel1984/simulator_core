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

// Use arrays as polynomial coefficients
type Polynomial []float64 

// Find value 
func (p Polynomial) Val(x float64) float64 {
  res := p[0]
  for i := 1; i < len(p); i++ {
    res = res * x + p[i] 
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

// Derivative
/*
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
*/

// Speed profile prepresentation
type Profile struct {
  State [3]Polynomial // [lift off, travel, set down] 
  Time  []float64     // time markers 
  T     float64       // total period (minimal)
  D     float64       // total length 
}

// Find trapez profile for distance d with limits vmax, amax
func Trapez(d, vmax, amax float64) *Profile {
  var p Profile 
  ta := vmax / amax
  tb := d / vmax 
  
  p.D = d
  p.State[0] = Polynomial{0.5*amax, 0, 0}
  p.State[2] = Polynomial{-0.5*amax, 0, 0} 
  
  if tb < ta {
    // triangle 
    ta = math.Sqrt(d / amax)
    tb = ta 
    vmax = ta * amax    
  } else {
    // trapez
    p.State[1] = Polynomial{vmax, (0.5*amax*ta-vmax)*ta} 
  }
  p.T = ta + tb
  p.State[2][1] = (vmax + amax*tb)
  p.State[2][2] = d - 0.5*amax*p.T*p.T
  p.Time = []float64{ta, tb, p.T} 
  
  return &p 
}

// Calculate state at time t for the given period 
func (p *Profile) At(t, period float64, res []float64) bool {
  if t < 0 || period < p.T {
    return false
  }
  k := (t / period) * p.T    // scaling
  for i,tm := range p.Time {
    if k <= tm {
      poly := p.State[i]
      res[0] = poly.Val(k) / p.D  // normalized position
      res[1] = poly.Val1d(k)  // velocity
      res[2] = poly.Val2d(k)  // acceleration
      return true
    }
  }
  return false
}

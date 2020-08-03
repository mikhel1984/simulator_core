package rigid

import (
  "math"
//  "gonum.org/v1/gonum/mat"
)

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

type Path struct {
  Joints  [][]float64 
  S       []float64    // when S != nil use it for steps
}

// Find closest index for a given s
func (p Path) Ind(s float64) int {
  if s < 0 || s > 1 {
    return -1
  }
  var res int
  n := len(p.Joints) - 1
  if p.S == nil {
    // uniform steps
    k := s * float64(n)
    kk := math.Floor(k) 
    k -= kk 
    if k <= 0.5 {
      res = int(kk)
    } else {
      res = int(kk) + 1
    }
  } else {
    // explicit steps
    var iup, idown int = n, 0
    for iup - idown > 1 {
      del := (iup - idown) / 2
      if del == 0 {
        del = 1
      }
      if p.S[iup-del] >= s {
        iup -= del
      } else if p.S[idown+del] <= s {
        idown += del
      }
    }
    if p.S[iup] - s < s - p.S[idown] {
      res = iup
    } else {
      res = idown
    }
  }
  return res
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

// square interpolation 
// 0 <= s <= 1
// return result and interval for s
func (p Path) GetSquare(s float64, res []Polynomial) (bool, float64, float64) {
  n := len(p.Joints)-1 
  m := p.Ind(s) 
  if m == -1 {
    return false, 0, 0
  }
  // need 3 points
  if m == 0 {
    m += 1
  } else if m == n {
    m -= 1
  }
  // step values
  var s1, s2, s3 float64
  if p.S == nil {
    del := 1 / float64(n) 
    s2 = del * float64(m)
    s1 = s2 - del
    s3 = s2 + del 
  } else {
    s1, s2, s3 = p.S[m-1], p.S[m], p.S[m+1]
  }
  s11, s22, s33 := s1*s1, s2*s2, s3*s3
  denom := (s22*s3-s33*s2) - (s11*s3-s33*s1) + (s11*s2-s22*s1)
  // approximation
  for i := 0; i < len(res); i++ {
    q1, q2, q3 := p.Joints[m-1][i], p.Joints[m][i], p.Joints[m+1][i] 
    res[i][0] = ((q2*s3-q3*s2) - (q1*s3-q3*s1) + (q1*s2-q2*s1)) / denom                 // a
    res[i][1] = ((s22*q3-s33*q2) - (s11*q3-s33*q1) + (s11*q2-s22*q1)) / denom           // b
    res[i][2] = ((s22*s3-s33*s2)*q1 - (s11*s3-s33*s1)*q2 + (s11*s2-s22*s1)*q3) / denom  // c    
  }
  return true, s1, s3
}



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
  p.State[0] = Polynomial{0.5*amax / d, 0, 0}    // normalize with total distance
  p.State[2] = Polynomial{-0.5*amax / d, 0, 0}
  ts := ta + tb
    
  if tb < ta {
    // triangle 
    ta = math.Sqrt(d / amax)
    tb = ta 
    vmax = ta * amax
    ts = ta + tb
  } else {
    // trapez
    p.State[1] = Polynomial{vmax*ts / d, (0.5*amax*ta-vmax)*ta / d} 
  }
  p.T = ts
  p.State[0][0] *= ts*ts
  p.State[2][0] *= ts*ts
  p.State[2][1] = (vmax + amax*tb) * ts / d
  p.State[2][2] = 1 - 0.5*amax*ts*ts / d
  p.Time = []float64{ta / ts, tb / ts, 1} 
  
  return &p 
}

// Calculate state at relative time k, return absolute time
func (p *Profile) At(k, period float64, res []float64) float64 {
  if k < 0 || k > 1 {
    res[0] = 0
    res[1] = 0
    res[2] = 0
  } else {
    for i,tm := range p.Time {
      if k <= tm {
        poly := p.State[i]
        res[0] = poly.Val(k)                       // relative position
        res[1] = poly.Val1d(k) / period            // velocity
        res[2] = poly.Val2d(k) / (period * period) // acceleration
        break
      }
    }
  }
  return k * period
}

// Calculate state for relative time and expanded period, return absolute time
func (p *Profile) AtScale(k, factor float64, res []float64) float64 {
  return p.At(k, factor * p.T, res) 
}

// function ik (pos, orient, q) (bool) 
// pos, orient - desirable values 
// q - current state, return new state here
// bool - result of evaluation

//func (base *Link) LinRot(qBeg []float64, pEnd, rEnd *mat.Dense, n int, 
//              func ik([]float64, *mat.Dense, *mat.Dense) bool) Path {
//  var pt Path 
//  
//}
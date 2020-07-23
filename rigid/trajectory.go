package rigid

import (
  "math"
)

type Path struct {
  Joints  [][]float64 
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
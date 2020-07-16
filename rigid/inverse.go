package rigid 

import (
  "gonum.org/v1/gonum/mat"
  "math"
)

type Ik6_Geometry struct {
  A  [3]float64
  B  float64
  C  [5]float64
  Dq [6]float64  // "deflections" in joints
  Q  *mat.Dense  // matrix of solutions, each solution in separate column
}

// TODO extract Ik6_Geometry from the robot tree 


// Inverse kinematics
// Return matrix with 8 solutions
func (par *Ik6_Geometry) IkFull(rot, pos *mat.Dense) *mat.Dense {
  // axis intersection point
  desPos := mat.DenseCopyOf(rot.Slice(0, 3, 2, 3))
  desPos.Scale(par.C[4], desPos)
  desPos.Sub(pos, desPos) // pos - c4 * rot * [0 0 1]^T

  // auxilary
  cx, cy, cz := desPos.At(0, 0), desPos.At(1, 0), desPos.At(2, 0)
  nx := math.Sqrt(cx*cx+cy*cy-par.B*par.B) - par.A[1]
  s1 := nx*nx + (cz-par.C[1])*(cz-par.C[1])
  s2 := (nx+2*par.A[1])*(nx+2*par.A[1]) + (cz-par.C[1])*(cz-par.C[1])
  k  := par.A[2]*par.A[2] + par.C[3]*par.C[3]

  // save results
  res := par.Q   
  
  // joint 1
  theta := math.Atan2(cy, cx) - math.Atan2(par.B, nx+par.A[1])
  res.Set(0, 0, theta)
  res.Set(0, 1, theta)
  res.Set(0, 4, theta)
  res.Set(0, 5, theta)
  theta = math.Atan2(cy, cx) + math.Atan2(par.B, nx+par.A[1]) - math.Pi
  res.Set(0, 2, theta)
  res.Set(0, 3, theta)
  res.Set(0, 6, theta)
  res.Set(0, 7, theta)

  // joint 2
  theta = -math.Acos((s1+par.C[2]*par.C[2]-k)/(2*math.Sqrt(s1)*par.C[2])) + math.Atan2(nx, cz-par.C[1])
  res.Set(1, 0, theta)
  res.Set(1, 4, theta)
  theta = math.Acos((s1+par.C[2]*par.C[2]-k)/(2*math.Sqrt(s1)*par.C[2])) + math.Atan2(nx, cz-par.C[1])
  res.Set(1, 1, theta)
  res.Set(1, 5, theta)
  theta = -math.Acos((s2+par.C[2]*par.C[2]-k)/(2*math.Sqrt(s2)*par.C[2])) + math.Atan2(nx+2*par.A[1], cz-par.C[1])
  res.Set(1, 2, theta)
  res.Set(1, 6, theta)
  theta = math.Acos((s2+par.C[2]*par.C[2]-k)/(2*math.Sqrt(s2)*par.C[2])) + math.Atan2(nx+2*par.A[1], cz-par.C[1])
  res.Set(1, 3, theta)
  res.Set(1, 7, theta)

  // joint 3
  theta = math.Acos((s1-par.C[2]*par.C[2]-k)/(2*par.C[2]*math.Sqrt(k))) - math.Atan2(par.A[2], par.C[3])
  res.Set(2, 0, theta)
  res.Set(2, 4, theta)
  theta = -math.Acos((s1-par.C[2]*par.C[2]-k)/(2*par.C[2]*math.Sqrt(k))) - math.Atan2(par.A[2], par.C[3])
  res.Set(2, 1, theta)
  res.Set(2, 5, theta)
  theta = math.Acos((s2-par.C[2]*par.C[2]-k)/(2*par.C[2]*math.Sqrt(k))) - math.Atan2(par.A[2], par.C[3])
  res.Set(2, 2, theta)
  res.Set(2, 6, theta)
  theta = -math.Acos((s2-par.C[2]*par.C[2]-k)/(2*par.C[2]*math.Sqrt(k))) - math.Atan2(par.A[2], par.C[3])
  res.Set(2, 3, theta)
  res.Set(2, 7, theta)

  // joint 4
  for col := 0; col < 3; col++ {
    cos1 := math.Cos(res.At(0, col))
    sin1 := math.Sin(res.At(0, col))
    cos23 := math.Cos(res.At(1, col) + res.At(2, col))
    sin23 := math.Sin(res.At(1, col) + res.At(2, col))
    res.Set(3, col, math.Atan2(rot.At(1, 2)*cos1-rot.At(0, 2)*sin1, rot.At(0, 2)*cos23*cos1+rot.At(1, 2)*cos23*sin1-rot.At(2, 2)*sin23))
    res.Set(3, 4+col, res.At(3, col)+math.Pi)
  }
  // joint 5
  for col := 0; col < 3; col++ {
    cos1 := math.Cos(res.At(0, col))
    sin1 := math.Sin(res.At(0, col))
    cos23 := math.Cos(res.At(1, col) + res.At(2, col))
    sin23 := math.Sin(res.At(1, col) + res.At(2, col))
    mp := rot.At(0, 2)*sin23*cos1 + rot.At(1, 2)*sin23*sin1 + rot.At(2, 2)*cos23
    res.Set(4, col, math.Atan2(math.Sqrt(1-mp*mp), mp))
    res.Set(4, 4+col, -res.At(4, col))
  }
  // joint 6
  for col := 0; col < 3; col++ {
    cos1 := math.Cos(res.At(0, col))
    sin1 := math.Sin(res.At(0, col))
    cos23 := math.Cos(res.At(1, col) + res.At(2, col))
    sin23 := math.Sin(res.At(1, col) + res.At(2, col))
    res.Set(5, col, math.Atan2(rot.At(0, 1)*sin23*cos1+rot.At(1, 1)*sin23*sin1+rot.At(2, 1)*cos23, -rot.At(0, 0)*sin23*cos1-rot.At(1, 0)*sin23*sin1-rot.At(2, 0)*cos23))
    res.Set(5, 4+col, res.At(5, col)-math.Pi)
  }

  return res
}

// Find closest solution for the previous state
// return -1 if solution not found
func (par *Ik6_Geometry) Closest(prev []float64) int {
  if prev == nil {
    prev = []float64{0,0,0,0,0,0} 
  }
  // find closest colution
  col := -1 
  minimal := math.Inf(1) 
  pi2 := 2*math.Pi
  for c := 0; c < 8; c++ {     // at most 8 solutions
    diff := 0.0
    for r := 0; r < 6; r++ {   // for 6 joint robot only
      q := par.Q.At(r,c)
      if math.IsNaN(q) {
        diff = math.Inf(1)
        continue 
      }
      // correct shift
      q -= par.Dq[r] 
      // correct range
      if q >= pi2 {
        q -= pi2
      } else if q <= -pi2 {
        q += pi2
      }
      par.Q.Set(r,c,q)
      // find maximal distance
      diff += math.Abs(q-prev[r])
    }
    // find minimal among maximum
    if diff < minimal {
      minimal = diff
      col = c
    }
  }
  return col 
}

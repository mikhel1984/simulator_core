package rigid 

import (
  //"gonum.org/v1/gonum/mat"
  "math"
  "../mat"
)

type Ik6_Geometry struct {
  // "standard" parameters
  A  [3]float64
  B  float64
  C  [5]float64
  // additional parameters
  Dq [6]float64  // "deflections" in joints
  Name [6]string // joint names 
  Q  *mat.Matrix  // matrix of solutions, each solution in separate column
  R  *mat.Matrix  // correct rotation 
}

// Inverse kinematics
// Return matrix with 8 solutions
// Based on Mathias Brandstotter, "An analytical solution of the inverse kinematics problem of industrial serial manipulators ..."
func (par *Ik6_Geometry) IkFull(rot, pos *mat.Matrix) {
  // axis intersection point 
  R := mat.Mul(rot, par.R) 
  //desPos := mat.DenseCopyOf(R.Slice(0, 3, 2, 3))
  desPos := pos.Copy()
  desPos.Sub(R.Col(2).Copy().Xk(par.C[4]))
  //desPos.Xk(par.C[4])
  //desPos.Sub(pos, desPos) // pos - c4 * R * [0 0 1]^T

  // auxilary
  cx, cy, cz := desPos.Get(0, 0), desPos.Get(1, 0), desPos.Get(2, 0)
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
    sin1, cos1 := math.Sincos(res.Get(0,col))
    sin23, cos23 := math.Sincos(res.Get(1, col) + res.Get(2, col))
    res.Set(3, col, math.Atan2(R.Get(1, 2)*cos1-R.Get(0, 2)*sin1, R.Get(0, 2)*cos23*cos1+R.Get(1, 2)*cos23*sin1-R.Get(2, 2)*sin23))
    res.Set(3, 4+col, res.Get(3, col)+math.Pi)
  }
  // joint 5
  for col := 0; col < 3; col++ {
    sin1, cos1 := math.Sincos(res.Get(0, col))
    sin23, cos23 := math.Sincos(res.Get(1, col) + res.Get(2, col))
    mp := R.Get(0, 2)*sin23*cos1 + R.Get(1, 2)*sin23*sin1 + R.Get(2, 2)*cos23
    res.Set(4, col, math.Atan2(math.Sqrt(1-mp*mp), mp))
    res.Set(4, 4+col, -res.Get(4, col))
  }
  // joint 6
  for col := 0; col < 3; col++ {
    sin1, cos1 := math.Sincos(res.Get(0, col))
    sin23, cos23 := math.Sincos(res.Get(1, col) + res.Get(2, col))
    res.Set(5, col, math.Atan2(R.Get(0, 1)*sin23*cos1+R.Get(1, 1)*sin23*sin1+R.Get(2, 1)*cos23, -R.Get(0, 0)*sin23*cos1-R.Get(1, 0)*sin23*sin1-R.Get(2, 0)*cos23))
    res.Set(5, 4+col, res.Get(5, col)-math.Pi)
  }
  
  // update joint state 
  pi2 := 2*math.Pi
  for c := 0; c < 8; c++ {
    for r := 0; r < 6; r++ {
      q := par.Q.Get(r,c)
      if math.IsNaN(q) {
        continue 
      }      
      // correct shift
      q += par.Dq[r] 
      // correct range
      if q >= pi2 {
        q -= pi2
      } else if q <= -pi2 {
        q += pi2
      }
      par.Q.Set(r,c,q)
    }
  }
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
  for c := 0; c < 8; c++ {     // at most 8 solutions
    diff := 0.0
    for r := 0; r < 6; r++ {   // for 6 joint robot only
      q := par.Q.Get(r,c)
      if math.IsNaN(q) {
        diff = math.Inf(1)
        continue 
      }
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

// Find closest solution based on joint map 
func (par *Ik6_Geometry) ClosestTo(qs map[string][]float64) int {
  prev := []float64{0,0,0,0,0,0}
  for i,nm := range par.Name {
    prev[i] = qs[nm][0] 
  }
  return par.Closest(prev) 
}

// Save solution into joint map 
func (par *Ik6_Geometry) SetTo(qs map[string][]float64, col int) {
  for i,nm := range par.Name {
    qs[nm][0] = par.Q.Get(i,col) 
  }
}

// Find parameters if the robot IK can be calculated
// via analytical solution for 6 joints 
func (base *Link) FindIk6Param(ee *Link) *Ik6_Geometry {
  mov := ee.Predecessors()
  if len(mov) != 6 {
    // sequence of 6 movable joints is expected
    return nil 
  }  
  // parse joint data 
  var par Ik6_Geometry
  for i, jnt := range mov {    
    if disp,ok := jnt.Src.Get6ikDeflection(); ok {
      par.Dq[i] = disp 
      par.Name[i] = jnt.Src.Name 
    } else {
      return nil
    }
  }  
  // update system state  
  qs := MakeJointMap(mov) 
  for i,nm := range par.Name {
    qs[nm][0] = par.Dq[i] 
  }  
  base.UpdateState(qs) 
  // a1, c1 
  pos := mov[1].Child.State.Pos   // second joint position 
  par.A[1] = pos.Get(0,0)
  par.C[1] = pos.Get(2,0)
  // c2 
  diff := mov[2].Child.State.Pos.Copy() 
  diff.Sub(pos)
  par.C[2] = diff.Get(2,0)
  // c3, a2 
  //pos = mov[4].Child.State.Pos
  diff = mov[4].Child.State.Pos.Copy()
  diff.Sub(mov[2].Child.State.Pos) 
  par.C[3] = diff.Get(2,0)
  par.A[2] = diff.Get(0,0) 
  // c4, b 
  diff = ee.State.Pos.Copy()
  diff.Sub(mov[4].Child.State.Pos)
  par.C[4] = diff.Get(2,0)
  par.B = -pos.Get(1,0) 
  
  par.Q = mat.New(6,8,nil) 
  par.R = ee.State.Rot.T().Copy() 
  
  return &par 
}



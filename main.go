package main 

import (
  "fmt"
  "./urdf"
  "./rigid"
)

func main() {

  model, err := urdf.GetFromFile("models/fanuc.urdf")
  if err != nil {
    fmt.Println(err)
    return 
  }
  
  base:= rigid.BodyTree(model)
  //println(base.Src.Name)
  
  qs := make(map[string][]float64)
  /*
  qs["joint1"] = []float64{ 0.1,0.5,1}
  qs["joint2"] = []float64{-0.2,0.5,1}
  qs["joint3"] = []float64{ 0.3,0.5,1}
  qs["joint4"] = []float64{-0.4,0.5,1}
  qs["joint5"] = []float64{ 0.5,0.5,1}
  qs["joint6"] = []float64{-0.6,0.5,1} 
  */
  qs["joint1"] = []float64{0.1,0,0}
  qs["joint2"] = []float64{0,0,0}
  qs["joint3"] = []float64{-1.5708,0,0}
  qs["joint4"] = []float64{0,0,0}
  qs["joint5"] = []float64{0.1,0,0}
  qs["joint6"] = []float64{0,0,0} 
 
  ee := base.Find("link7") 
    
  //base.UpdateState(qs)
  //ee.State.Rot.Print()
  //ee.State.Pos.Print()
  //rigid.MatPrint(ee.State.Rot)
  //fmt.Println("")
  //rigid.MatPrint(ee.State.Pos) 
  
  //lst := ee.Predecessors() 
  //jac := ee.Jacobian(lst) 
  //rigid.MatPrint(jac) 
  //base.UpdateDyn(9.81) 
  //tau := rigid.ReadTorques(lst) 
  //rigid.MatPrint(tau)
  
  // inverse kinematics 
  //prev := rigid.MakeJointMap(lst)  
  par := base.FindIk6Param(ee)
  //fmt.Println(par.A)
  //fmt.Println(par.B)
  //fmt.Println(par.C)
  base.UpdateState(qs)
  par.IkFull(ee.State.Rot, ee.State.Pos)
  //rigid.MatPrint(ee.State.Pos)
  //rigid.MatPrint(ee.State.Rot)
  rigid.MatPrint(par.Q) 
  n := par.ClosestTo(qs)
  println()
  println(n)
  
  tr := rigid.Trapez(10, 3, 2)
  fmt.Println(tr.State[0])
  fmt.Println(tr.State[1])
  fmt.Println(tr.State[2])
  t0 := tr.Time[0]  
  fmt.Println(tr.Time)
  t1 := tr.T
  t2 := t1*2 
  pt := []float64{0,0,0}
  
  tr.At(t0, t1, pt)
  println()
  fmt.Println(pt)
  
  tr.At(t0, t2, pt)
  fmt.Println(pt)
  
}
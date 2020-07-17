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
  qs["joint1"] = []float64{ 0.1,0.5,1}
  qs["joint2"] = []float64{-0.2,0.5,1}
  qs["joint3"] = []float64{ 0.3,0.5,1}
  qs["joint4"] = []float64{-0.4,0.5,1}
  qs["joint5"] = []float64{ 0.5,0.5,1}
  qs["joint6"] = []float64{-0.6,0.5,1} 
 
  ee := base.Find("link7") 
  println(ee)  
  
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
  fmt.Println(par.A)
  fmt.Println(par.B)
  fmt.Println(par.C)
  base.UpdateState(qs)
  par.IkFull(ee.State.Rot, ee.State.Pos)
  rigid.MatPrint(par.Q) 
  //n := par.ClosestTo(prev) 
  
  //println(n)
  
}
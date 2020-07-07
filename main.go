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
  println(base.Src.Name)
  
  qs := make(map[string][3]float64) 
  qs["joint1"] = [3]float64{0.1,1,0}
  qs["joint2"] = [3]float64{-0.2,1,0}
  qs["joint3"] = [3]float64{0.3,1,0}
  qs["joint4"] = [3]float64{-0.4,1,0}
  qs["joint5"] = [3]float64{0.5,1,0}
  qs["joint6"] = [3]float64{-0.6,1,0} 
 
  ee := base.Find("link7") 
  println(ee)  
  
  base.UpdateState(qs)
  //ee.State.Rot.Print()
  //ee.State.Pos.Print()
  //rigid.MatPrint(ee.State.Rot)
  //fmt.Println("")
  //rigid.MatPrint(ee.State.Pos) 
  
  lst := ee.Predecessors() 
  //jac := ee.Jacobian(lst) 
  //rigid.MatPrint(jac) 
  base.UpdateDyn(9.81) 
  tau := rigid.ReadTorques(lst) 
  rigid.MatPrint(tau)
  
}
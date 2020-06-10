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
  
  for i := 0; i < len(model.Links); i++ {
    fmt.Println(model.Links[i].Name)
  }
  for i := 0; i < len(model.Joints); i++ {
    fmt.Println(model.Joints[i].Name)
  }   
  
  lnk := rigid.MakeTree(model)
  print(lnk.Src.Name)
}
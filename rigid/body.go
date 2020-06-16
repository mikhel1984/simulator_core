package rigid 

import (
  "../urdf"
)

/* func stringToList(s string) []float64 {
  nums := strings.Split(s," ")
  res := make([]float64,3,3) 
  for i := 0; i < len(nums); i++ {
    res[i],_ = strconv.ParseFloat(nums[i], 64)
  }
  return res 
} */

type Link struct {
  // source 
  Src          *urdf.Link 
  // tree 
  Joints       []*Joint 
  Parent       *Joint 
  // current state
  State         Transform
} 

type Joint struct {
  // source 
  Src          *urdf.Joint 
  // tree 
  Parent       *Link 
  Child        *Link 
  // type 
  Type          JointType
  // current angle
  Angle         float64 
  Velocity      float64
  Acceleration  float64 
  // constant transformation 
  Trans         Transform 
}

func MakeTree(model *urdf.Model) *Link {
  // read links 
  links := make(map[string]*Link)   
  for i := 0; i < len(model.Links); i++ {
    lnk := new(Link)  
    lnk.Src = &model.Links[i]
    links[ model.Links[i].Name] = lnk     
  }
  // joints   
  for i := 0; i < len(model.Joints); i++ {
    jnt := new(Joint)
    jnt.Src = &model.Joints[i]
    lnk := links[model.Joints[i].Parent.Name] 
    jnt.Parent = lnk     
    lnk.Joints = append(lnk.Joints, jnt) 
    
    lnk = links[model.Joints[i].Child.Name]
    lnk.Parent = jnt
    jnt.Child = lnk     
  }
  // find "free" link 
  for _,v := range links {
    if v.Parent == nil {
      return v       
    }
  }
  
  return nil
} 


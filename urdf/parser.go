package urdf 

import (
    "encoding/xml"    
    "io/ioutil"
    "os"
    "strings"
    "strconv" 
)

func stringToList(s string) []float64 {
  nums := strings.Split(s," ")
  res := make([]float64,3,3) 
  for i := 0; i < len(nums); i++ {
    res[i],_ = strconv.ParseFloat(nums[i], 64)
  }
  return res 
} 

type Model struct {
  XMLName xml.Name `xml:"robot"`
  Joints []Joint   `xml:"joint"`
  Links  []Link    `xml:"link"`
}

/* func (m *Model) ParseData() {
  for i := 0; i < len(m.Joints); i++ {
    m.Joints[i].parseData()
  }
  for i := 0; i < len(m.Links); i++ {
    m.Links[i].parseData() 
  }
}
 */
type Joint struct {
  XMLName xml.Name `xml:"joint"` 
  Type    string   `xml:"type,attr"`
  Name    string   `xml:"name,attr"` 
  Origin  Origin_   `xml:"origin"`
  Parent  Parent   `xml:"parent"`
  Child   Child    `xml:"child"` 
  Axis    Axis_    `xml:"axis"`
  Limit   Limit_    `xml:"limit"` 
  Dynamics Dynamics `xml:"dynamics"` 
  General6ik string `xml:"general6ik"` // Define initial configuration if generalized approach can be applied
}

func (v *Joint) Get6ikDeflection() (float64, bool) {
  num, res := strconv.ParseFloat(v.General6ik, 64)
  if res != nil {
    return 0, false 
  }
  return num, true 
}

/* func (v *Joint) parseData() {
  v.Origin.parseData()
  v.Axis.parseData() 
  v.Limit.parseData() 
  v.Dynamics.parseData()
} */

type Origin_ struct {
  XMLName xml.Name `xml:"origin"`
  Xyz     string   `xml:"xyz,attr"`
  Rpy     string   `xml:"rpy,attr"`  
}

func (v *Joint) GetXyz() []float64 {
  return stringToList(v.Origin.Xyz) 
}

func (v *Joint) GetRpy() []float64 {
  return stringToList(v.Origin.Rpy) 
}

/* func (v *Origin_) parseData() {
  v.Xyz = stringToList(v.xyz) 
  v.Rpy = stringToList(v.rpy)
} */

type Parent struct {
  XMLName xml.Name `xml:"parent"`
  Name    string   `xml:"link,attr"`
}

type Child struct {
  XMLName xml.Name `xml:"child"` 
  Name    string   `xml:"link,attr"` 
}

type Axis_ struct {
  XMLName xml.Name `xml:"axis"`
  Xyz     string   `xml:"xyz,attr"`
}

func (v *Joint) GetAxis() int {
  nums := strings.Split(v.Axis.Xyz," ") 
  for i := 0; i < len(nums); i++ {
    if nums[i] == "1" {
      return i
    }
  }
  return 0
} 

type Limit_ struct {
  XMLName xml.Name `xml:"limit"`
  Effort  string   `xml:"effort,attr"`
  Lower   string   `xml:"lower,attr"`
  Upper   string   `xml:"upper,attr"`
  Velocity string  `xml:"velocity,attr"`
}

func (v *Joint) GetLimits() (float64,float64) {
  lo,_ := strconv.ParseFloat(v.Limit.Lower,64) 
  up,_ := strconv.ParseFloat(v.Limit.Upper,64)
  return lo, up
}

/* func (v *Limit_) parseData() {
  v.Effort,_ = strconv.ParseFloat(v.effort,64)
  v.Lower,_ = strconv.ParseFloat(v.lower,64)
  v.Upper,_ = strconv.ParseFloat(v.upper,64)
  v.Velocity,_ = strconv.ParseFloat(v.velocity,64)
}
 */
type Dynamics struct {
  XMLName  xml.Name `xml:"dynamics"`
  Damping  string   `xml:"damping,attr"`
  Friction string  `xml:"friction,attr"`
}

/* func (v *Dynamics) parseData() {
  v.Damping,_ = strconv.ParseFloat(v.damping,64)
  v.Friction,_ = strconv.ParseFloat(v.friction,64)
} */

type Link struct {
  XMLName xml.Name `xml:"link"`
  Name    string   `xml:"name,attr"` 
  Visual  Visual   `xml:"visual"`
  Collision Collision `xml:"collision"` 
  Inertial Inertial_ `xml:"inertial"`  
}

func (l *Link) GetMass() float64 {
  m,_ := strconv.ParseFloat(l.Inertial.Mass.Value, 64) 
  return m
}

func (l *Link) GetMassCenter() []float64 {
  return stringToList(l.Inertial.Origin.Xyz) 
}

func (l *Link) GetInertia() []float64 {
  res := make([]float64,6,6)  
  res[0],_ = strconv.ParseFloat(l.Inertial.Inertia.Ixx,64)
  res[1],_ = strconv.ParseFloat(l.Inertial.Inertia.Ixy,64)
  res[2],_ = strconv.ParseFloat(l.Inertial.Inertia.Ixz,64)
  res[3],_ = strconv.ParseFloat(l.Inertial.Inertia.Iyy,64)
  res[4],_ = strconv.ParseFloat(l.Inertial.Inertia.Iyz,64)
  res[5],_ = strconv.ParseFloat(l.Inertial.Inertia.Izz,64)
  return res 
}

/* func (l *Link) parseData() {
  l.Visual.parseData()
  l.Collision.parseData()
  l.Inertial.parseData() 
}
 */
type Visual struct {
  XMLName xml.Name `xml:"visual"`
  Origin  Origin_   `xml:"origin"`
  Geometry Geometry `xml:"geometry"` 
}

/* func (v *Visual) parseData() {
  v.Origin.parseData()
} */

type Collision struct {
  XMLName xml.Name `xml:"collision"`
  Origin  Origin_   `xml:"origin"`
  Geometry Geometry `xml:"geometry"` 
}

/* func (v *Collision) parseData() {
  v.Origin.parseData() 
} */

type Geometry struct {
  XMLName xml.Name `xml:"geometry"` 
  Mesh    Mesh     `xml:"mesh"`
}

type Mesh struct {
  XMLName xml.Name `xml:"mesh"`
  Name    string   `xml:"filename,attr"` 
}

type Inertial_ struct {
  XMLName xml.Name `xml:"inertial"`
  Mass    Mass_     `xml:"mass"`
  Origin  Origin_   `xml:"origin"` 
  Inertia Inertia  `xml:"inertia"` 
}

/* func (v *Inertial) parseData() {
  v.Mass.parseData()
  v.Origin.parseData()
  v.Inertia.parseData() 
} */

type Mass_ struct {
  XMLName xml.Name `xml:"mass"`
  Value   string   `xml:"value,attr"`
}

/* func (v *Mass) parseData() {
  v.Value,_ = strconv.ParseFloat(v.value,64) 
} */

type Inertia struct {
  XMLName xml.Name `xml:"inertia"`
  Ixx     string   `xml:"ixx,attr"`
  Ixy     string   `xml:"ixy,attr"`
  Ixz     string   `xml:"ixz,attr"`
  Iyy     string   `xml:"iyy,attr"`
  Iyz     string   `xml:"iyz,attr"`
  Izz     string   `xml:"izz,attr"`  
}



/* func (v *Inertia) parseData() {
  res := make([]float64,6,6)
  res[0],_ = strconv.ParseFloat(v.ixx,64)
  res[1],_ = strconv.ParseFloat(v.ixy,64)
  res[2],_ = strconv.ParseFloat(v.ixz,64)
  res[3],_ = strconv.ParseFloat(v.iyy,64)
  res[4],_ = strconv.ParseFloat(v.iyz,64)
  res[5],_ = strconv.ParseFloat(v.izz,64)
  v.Value = res 
} */

func GetFromFile(fname string) (*Model,error) {
  urdfFile, err := os.Open(fname)
    
  if err != nil {
    return nil, err 
  }
  defer urdfFile.Close() 
  
  byteValue, _ := ioutil.ReadAll(urdfFile)
  
  model := new(Model)    
  xml.Unmarshal(byteValue, model) 
  //model.ParseData() 
  
  return model, nil  
}

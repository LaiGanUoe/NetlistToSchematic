graph [
  node [
    id 0
    label "VDD"
    type "V"
    value "DC 15"
  ]
  node [
    id 1
    label "1"
    type "node"
  ]
  node [
    id 2
    label "0"
    type "ground"
  ]
  node [
    id 3
    label "J1"
    type "J"
    value "JFET_N"
  ]
  node [
    id 4
    label "2"
    type "node"
  ]
  node [
    id 5
    label "R1"
    type "R"
    value "10"
  ]
  edge [
    source 0
    target 1
  ]
  edge [
    source 0
    target 2
  ]
  edge [
    source 1
    target 3
  ]
  edge [
    source 2
    target 3
  ]
  edge [
    source 2
    target 5
  ]
  edge [
    source 3
    target 4
  ]
  edge [
    source 4
    target 5
  ]
]

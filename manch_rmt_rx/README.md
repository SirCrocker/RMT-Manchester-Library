### Manchester RMT RX Module

Made for receiving Manchester-encoded data using the RMT module of the ESP32 microcontrollers. A custom protocol
is set on top of the manchester symbols so the data can be received without major problems; the protocol is as follows:


```mermaid
block-beta
    columns 1
    
    block:main:1
        columns 5
        header["Header"]:1
        data["Data"]:3
        trailer["Trailer"]:1
    end

    style main fill:#0000,stroke-width:0px

    block:desc:5
        columns 5
        h["Four times the 'header\nsymbol' that is not a valid\nmanchester sequence."]:1
        d["The data has to be a multiple of bytes."]:3
        t["Symbols that MUST NOT be\na valid manchester sequence."]:1
    end

    header --> h
    data --> d
    trailer --> t

    style desc fill:#0000,stroke-width:0px,height:0px
    style t fill:#0000,stroke-width:0px
    style d fill:#0000,stroke-width:0px
    style h fill:#0000,stroke-width:0px             
```

### Header symbol:

![Manchester header with a clock signal for timing visualization](wavedrom.png "Manchester Header")
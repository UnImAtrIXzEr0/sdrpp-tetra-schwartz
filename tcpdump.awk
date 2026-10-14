 BEGIN {
        sending = 0;
        count = 0;
	print("begin")
    }
    /^    192.168.44.1[ \.]/ {
	sending = 1;
    }
    /^    192.168.44.100\./ {
	sending = 0;
    }
    (/0x0020:/ || /0x0220:/) && sending {
    	r0 = strtonum("0x" substr($5, 3, 2))
    	reg = int(r0/2)
    	tx = (reg*2 != r0)
    	data[reg] = $6$7
    	count = count+1
    }
    count > 200 {
    	count = 0
    	z = ""
    	for (k in data) {
    		if (k >= 3 && k <=8) {
    			continue
    		}
    		if (data[k] == "00989680") {
    			continue;
    		}
    		z = z k ":" data[k] " "
    	}
    	print "TX:" tx "  " z
    }
    END {
    } 
    
    
       

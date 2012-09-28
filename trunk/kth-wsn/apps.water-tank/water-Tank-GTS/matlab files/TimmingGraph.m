S1op1=0;S1op2=0; S1op3=0;
S2op1=0;S2op2=0; S2op3=0;
S3op1=0;S3op2=0; S3op3=0;
S4op1=0;S4op2=0; S4op3=0;
S5op1=0;S5op2=0; S5op3=0;
S6op1=0;S6op2=0; S6op3=0;

S1gtson=0; S2gtson=0; S3gtson=0;
S4gtson=0; S5gtson=0;S6gtson=0;

   if(lock==1&& S1==1)
     S1op1=1; 
    end

   if(lock==1&& S2==1)
      S2op1=1; 
     
    end

   if(lock==1&& S3==1)
     S3op1=1
    end

   if(lock==1&& S4==1)
      S4op1=1; 
    end

   if(lock==1&& S5==1)
     S5op1=1; 
    end

   if(lock==1&& S6==1)
      S6op1=1;
    end    

if(lock==0)

if(S1==1)
S1op2=1;
S1gtson=1;
end

if(S2==1)
S2op2=1;
S2gtson=1;
end

if(S3==1)
S3op2=1;
S3gtson=1;
end

if(S4==1)
S4op2=1;
S4gtson=1;
end

if(S5==1)
S5op2=1;
S5gtson=1;
end

if(S6==1)
S6op2=1;
S6gtson=1;
end
 
end

    


if(S1==1&&S1gtson==1)
   S1op3=1;
end

if(S2==1&&S2gtson==1)
   S2op3=1;
end
if(S3==1&&S3gtson==1)
   S3op3=1;
end
if(S4==1&&S4gtson==1)
   S4op3=1;
end
if(S5==1&&S5gtson==1)
   S5op3=1;
end
if(S6==1&&S6gtson==1)
   S6op3=1;
end

function [Q_final,H_final,LAMBDAlog,Qlog]= comp_Q_H(Qsel,data)

for k=1:data.sizeHORIZON
    
    N=sum(Qsel(:,k));
    M=length(Qsel(:,k));
    
    Q=zeros(N,M);
    
    l=1;
    for m=1:M
        if Qsel(m,k)==1;
            Q(l,m)=1;
            l=l+1;
        end
    end
    
    clear lambda
    
    for j=1:data.nSensors
        lambda(j,j)=rand;
        if lambda(j,j)<data.lossP
            lambda(j,j)=1;
        else
            lambda(j,j)=0;
        end
    end
    
    if (size(Q,1) > 0)
        Z=Q*lambda*Q';

        P=sum(diag(Z));
        H=zeros(P,N);

        l=1;
        for m=1:size(Z,1)
            if Z(m,m)==1;
                H(l,m)=1;
                l=l+1;
            end
        end
    else
        H = zeros(0,0);
    end
    
    H_final{1,k}=H;
    Q_final{1,k}=Q;
    LAMBDAlog(:,k)=diag(lambda);
    Qlog(:,k)=diag(Q'*Q);
end
end

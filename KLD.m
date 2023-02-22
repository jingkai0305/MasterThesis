function score = KLD(m1,m2,cov1,cov2)
    score = 0.5*( (m2-m1)'*inv(cov2)*(m2-m1) + trace(inv(cov2)*cov1)...
        - log(det(cov1)/det(cov2)) - 2 );
end
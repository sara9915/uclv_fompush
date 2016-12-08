function n_f = m_max_funct(obj, nu, m)     
    n_f_integrand = @(p1,p2) (nu*m*Helper.g/obj.A) * sqrt([p1;p2;0]'*[p1;p2;0]);
    n_f = Helper.DoubleGaussQuad(n_f_integrand,-obj.a/2,obj.a/2,-obj.b/2,obj.b/2);
end
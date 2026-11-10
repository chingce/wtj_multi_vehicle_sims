def timeGap(sn,vn,sndot,alpha,tau,gmin,lam):
    return alpha * (sn - tau*vn - gmin) + lam*sndot

def wtj(sn,vn,sndot,alpha,tau_min,tau_max,gmin,lam,c):

    al = alpha * (sn - tau_max*vn - gmin) + lam*min(0,sndot)
    au = alpha * (sn - tau_min*vn - gmin) + lam*min(0,sndot)
    a_star = 0
    if vn <= 0:
        return al;
    
    a_star = c*sqrt(1/(vn + 0.25)) - (c*sqrt(1/(30 + 0.25)))
    return min(max(al,a_star), au)


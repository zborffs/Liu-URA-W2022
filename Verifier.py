import gurobipy as gp

model=[]
model = gp.Model("qp")

#Define Decision Variables
Xti=model.addVars(nt,n,ns, lb=(-radius*np.ones((nt,n,ns))).tolist(),ub=(radius*np.ones((nt,n,ns))).tolist(),name="Xti")
Xt=model.addVars(nt+1,n,lb=(-radius*np.ones((nt+1,n))).tolist(),ub=(radius*np.ones((nt+1,n))).tolist(),name="Xt")

Uti = model.addVars(nt, ns,lb=(-u_lim*np.ones((nt,ns))).tolist(),ub=(u_lim*np.ones((nt,ns))).tolist(), vtype=GRB.CONTINUOUS, name='Uti')
Ut = model.addVars(nt,lb=(-u_lim*np.ones(nt)).tolist(),ub=(u_lim*np.ones(nt)).tolist(), vtype=GRB.CONTINUOUS, name='Ut')

Mu = model.addVars(nt,ns, vtype=GRB.BINARY,name="Mu")
Dti=model.addVars(nt,n,ns, lb=(-inf*np.ones((nt,n,ns))).tolist(),ub=(inf*np.ones((nt,n,ns))).tolist(),name="Dti")
# if P is None: P=np.eye(2*n)
P1=P[:n,:n]  ; P2=P[:n,n:] ; P3=P[n:,n:]

#Objective:
obj=gp.quicksum([ \
      gp.quicksum(Xt[1,j] * P1[j,k] * Xt[1,k] for j in range(n) for k in range(n) ),
      gp.quicksum(Xt[2,j] * P3[j,k] * Xt[2,k] for j in range(n) for k in range(n) ),
      gp.quicksum(2*Xt[1,j] * P2[j,k] * Xt[2,k] for j in range(n) for k in range(n) ),
      gp.quicksum(-Xt[0,j] * P1[j,k] * Xt[0,k] for j in range(n) for k in range(n) ),
      gp.quicksum(-Xt[1,j] * P3[j,k] * Xt[1,k] for j in range(n) for k in range(n) ),
      gp.quicksum(-2*Xt[0,j] * P2[j,k] * Xt[1,k] for j in range(n) for k in range(n) ) ])
# obj=(Xt[1,...] @ P[:n,:n] @ Xt[1,...]+2*(Xt[1,...] @ P[:n,n:] @ Xt[2,...])+Xt[2,...] @ P[n:,n:] @ Xt[2,...])\
#  - (Xt[0,...] @ P[:n,:n] @ Xt[0,...]+2*(Xt[0,...] @ P[:n,n:] @ Xt[1,...]) + Xt[1,...] @ P[n:,n:] @ Xt[1,...])


#constraints
#Guards:
#states
Xlow_ctr=[model.addLConstr(Xti[t,j,i]>=guards[s[i],j,0]*Mu[t,i],name='xlow_{}_{}_{}'.format(i,j,t)) \
                                for i in range(ns) for j in range(n) for t in range(nt)]
Xupp_ctr=[model.addLConstr(Xti[t,j,i]<=guards[s[i],j,1]*Mu[t,i],name='xupp_{}_{}_{}'.format(i,j,t)) \
                                for i in range(ns) for j in range(n) for t in range(nt)]
#control
Ulow_ctr=[model.addLConstr(Uti[t,i]>=-u_lim*Mu[t,i],name='xlow_{}_{}'.format(i,t)) \
                                for i in range(ns)  for t in range(nt)]
Uupp_ctr=[model.addLConstr(Uti[t,i]<=u_lim*Mu[t,i],name='xupp_{}_{}'.format(i,t)) \
                                for i in range(ns) for t in range(nt)]

#Uncertainity bounds
Dlow_ctr=[model.addLConstr(Dti[t,j,i]>=-bounds[j,s[i]]*Mu[t,i],name='Dlow_{}_{}_{}'.format(i,j,t)) \
                                for i in range(ns) for j in range(n) for t in range(nt)]
Dupp_ctr=[model.addLConstr(Dti[t,j,i]<=bounds[j,s[i]]*Mu[t,i],name='Dupp_{}_{}_{}'.format(i,j,t)) \
                                for i in range(ns) for j in range(n) for t in range(nt)]


#Discrete System

dyn_constr = {(j, t): model.addLConstr(Xt[t+1,j]==gp.quicksum(gp.quicksum([gp.quicksum(A[s[i],j,k]*Xti[t,k,i] for k in range(n)) \
                        ,B[s[i],j,0]*Uti[t,i],Mu[t,i]*C[s[i],j,0],Dti[t,j,i]]) for i in range(ns)),name='dyn_{}_{}'.format(j,t)) for j in range(n) for t in range(nt)}
#Convex Hull constraints
[model.addLConstr(gp.quicksum(Xti[t,j,i] for i in range(ns)) == Xt[t,j] ,name='Xti_{}_{}'.format(j,t))\
                                for j in range(n) for t in range(nt)]
[model.addLConstr(gp.quicksum(Uti[t,i] for i in range(ns)) ==Ut[t],name='Uti_{}'.format(t))\
                                for t in range(nt)]
[model.addLConstr(gp.quicksum(Mu[t,i] for i in range(ns)) == 1,name='Mu_{}'.format(t))\
                                for t in range(nt)]
# [model.addLConstr(Mu[0,1]==1)]
# [model.addLConstr(Mu[1,4]==1)]
#Control Rule:
im=0
[model.addLConstr(Uti[t,i]==-(1/self.Objective.R[im,im])*gp.quicksum((Bc[s[i]].T @ self.P[s[i],1:,1:])[0,k] * Xti[t,k,i] for k in range(n)),name='Ctrl_Rule_{}_{}'.format(t,i))\
                                for t in range(nt) for i in range(ns)]

M_=0.1
model.addConstr(gp.quicksum([Xt[0,0]*Xt[0,0],Xt[0,1]*Xt[0,1]])>=(M_**2),name='Exc0_')
# model.addConstr(gp.quicksum([Xt[0,0]*Xt[0,0],Xt[0,1]*Xt[0,1]])<=0.01,name='Exc1_')

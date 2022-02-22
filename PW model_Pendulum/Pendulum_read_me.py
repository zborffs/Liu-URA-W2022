# Learning Pendulum using Piecewise Affine Syestems
# Instruction to load and use the learned PWA system:
#
# We set the domain of interest and grids as below, respectively for state x0 and x1:
#

Domain=[-pi,pi,-pi,pi]
Switching_setting={'Grid':[15,15],'Margin_in_percent':[0,0]}


#Then we assign an index number for any partition of the PW model:
Switching_Grid=Switching_setting['Grid']

# creates a (225,) numpy array with elements [0, 1, 2, 3, ..., 225], where 225 = 15 * 15 which is np.prod(switching_grid)
# then reshapes it to be 15x15
subsys_map=np.arange(np.prod(Switching_Grid)).reshape(Switching_Grid)

# For the PW system, I consider the following form for any piece.
# x_dot=W*Phi+Wc*Phi*u,
# where Phi=[1 x0 x1]'
#By putting together these weights, we can represent any mode of the system with the total weights Wt=[W Wc].
# Considering these weights for all pieces, we can use a 3d np array to store the weights for all 15*15 modes of the system.
# For this regard, dataset named "Weights.npy", can be loaded as

weights=np.load('PW model_Pendulum/Weights_updated1.npy')

# Then we can extract the weights for the active mode as below.
#The active mode is given by sigma. So to get the weights for the active mode:

weights_active=weights[sigma,...]

# Obviously, the oreder of indexes of different modes should be cosistent with the learned system.
# Otherwise, we will pickup the wrong set of weights for any mode. In this regard, the following routine is used
# to get the right index for the active mode for any given x.

def sigma_calc(self, x):
    # To calculate the piece number for given x without looking at the neigbors. So it is faster than sigma_calc_neighbors
    for i in range(self.Lib.n):
        self.target_index[i],_=divmod(x[i]-self.Domain[2*i], self.grid_len_list[i])
        if self.target_index[i]==self.Switching_Grid[i]: self.target_index[i]-=1
    return(self.subsys_map[tuple(self.target_index.astype(int))])

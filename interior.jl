using LinearAlgebra: Diagonal

# Objective function gradient and Hessian

obj_grad(x) = Float64[1, 0]
obj_hess(x) = Float64[0 0; 0 0]

# Constraint errors, gradients, and Hessians

c1_err(x) = 1 - 0.5*(x[1]^2 + x[2]^2)
c1_grad(x) = Float64[-x[1], -x[2]]
c1_hess(x) = Float64[-1 0; 0 -1]

c2_err(x) = x[1] + x[2]
c2_grad(x) = Float64[1, 1]
c2_hess(x) = zeros(2, 2)

# Constraint collection

c_err = [c1_err, c2_err]
c_grad = [c1_grad, c2_grad]
c_hess = [c1_hess, c2_hess]

# Helper functions

c_errs(x) = [f(x) for f=c_err]
c_grads(x) = vcat([transpose(f(x)) for f=c_grad]...)
c_hesss(x) = [f(x) for f=c_hess]

function resid(x)
	y = x[3:end] # constraint multipliers
	vcat(
		obj_grad(x) + transpose(c_grads(x)) * y,
		-(y .* c_errs(x))
	)
end

function step_mat(x)
	y = x[3:end] # constraint multipliers
	hess = obj_hess(x) + sum(y .* c_hesss(x))
	vcat(
		hcat(hess, transpose(c_grads(x))),
		hcat(-(y .* c_grads(x)), -Array(Diagonal(c_errs(x))))
	)
end

barrier(u) = Float64[0, 0, u, u]

step(x, u) = x + step_mat(x) \ (barrier(u) - resid(x))

x = Float64[0, 0, -0.5, -0.5]
# x = Float64[0.1, 0, -0.5, -0.5]
@show x, resid(x)

for i = 1:16
	u = 1/2^i
	global x = step(x, u)
	@show x, resid(x)
end

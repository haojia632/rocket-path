# Minimize f(x0, x1) = x0
# Constraints: x0 + x1 >= 0, x0 - x1 >= 0
# State vector is x0, x1, c0, c1
# c0, c1 are the constraint multipliers

using LinearAlgebra: Diagonal

objectiveGradient(x) = Float64[1, 0]

constraint_1_error(x) = x[1] + x[2]
constraint_1_gradients(x) = Float64[1, 1]

constraint_2_error(x) = x[1] - x[2]
constraint_2_gradients(x) = Float64[1, -1]

constraint_3_error(x) = 10 - x[1]
constraint_3_gradients(x) = Float64[-1, 0]

constraint_errors(x) = Float64[
	constraint_1_error(x),
	constraint_2_error(x),
	constraint_3_error(x)
]

constraint_gradients(x) = vcat(
	transpose(constraint_1_gradients(x)),
	transpose(constraint_2_gradients(x)),
	transpose(constraint_3_gradients(x))
)

function residuals(x)
	y = x[3:end] # constraint multipliers
	vcat(
		objectiveGradient(x) + transpose(constraint_gradients(x)) * y,
		-(y .* constraint_errors(x))
	)
end

function stepMatrix(x)
	y = x[3:end] # constraint multipliers
	vcat(
		hcat(zeros(2, 2), transpose(constraint_gradients(x))),
		hcat(-(y .* constraint_gradients(x)), -Array(Diagonal(constraint_errors(x))))
	)
end

barrier(t) = Float64[0, 0, 1/t, 1/t, 1/t]

step(x, t) = x + stepMatrix(x) \ (barrier(t) - residuals(x))

x = Float64[-2, 1, 1, 1, 1]
@show x
@show residuals(x)

for i = 1:16
	t = 2^i
	global x = step(x, t)
	@show x
end

@show residuals(x)

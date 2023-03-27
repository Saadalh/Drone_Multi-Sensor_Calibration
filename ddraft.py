list = []
hello = [1, 2, 4]
list.append(hello)
print((hello))
hellocopy = hello.copy()
hellocopy[0] = 10
list.append(hellocopy)
print((hellocopy))
print(list)

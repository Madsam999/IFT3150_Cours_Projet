with open("./data/assets/grid.15.bin", "rb") as file:
    data = file.read()

# Print each byte in hexadecimal format
for i, byte in enumerate(data):
    print(f"Byte {i}: {byte:02x}")

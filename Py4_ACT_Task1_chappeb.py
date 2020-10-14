# Ben Chappell ADD HEADER HERE

def main():
    lName = input("Enter your last name:\n")
    fName = input("Enter your first name:\n")
    age = int(input("Enter your age in whole years:\n"))
    days = int(input("Enter the number of days since your last birthday:\n"))

    name = fName + " " + lName
    years = age + (days / 365)
    minutes = int((age * 60 * 24 * 365.242199) + (days *  60 * 24))

    f = open("Py4_ACT_Task1_chappeb.txt", 'w')

    lines = [name + "\n", "You are " + str(years) + " years old.\n", "You are " + str(minutes) + " minutes old\n"]

    f.writelines(lines)

    f.close()

if __name__ == "__main__":
    main()
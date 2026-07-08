import tkinter as tk

class UserInterface:
    def __init__(self, master):
        self.master = master
        master.title("User Interface")

        self.label = tk.Label(master, text="Hello, World!")
        self.label.pack()

        self.button = tk.Button(master, text="Click Me", command=self.on_button_click)
        self.button.pack()

    def on_button_click(self):
        self.label.config(text="Button Clicked!")
if __name__ == "__main__":
    root = tk.Tk()
    ui = UserInterface(root)
    root.mainloop() 
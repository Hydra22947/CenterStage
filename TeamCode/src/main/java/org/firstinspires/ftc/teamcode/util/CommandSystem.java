package org.firstinspires.ftc.teamcode.util;

import java.util.HashMap;
import java.util.Map;
import java.util.Scanner;

public class CommandSystem {
    private final Map<String, Runnable> commands;

    public CommandSystem() {
        commands = new HashMap<>();
    }

    public void addCommand(String commandName, Runnable function) {
        commands.put(commandName, function);
    }

    public void executeCommand(String commandName) {
        Runnable function = commands.get(commandName);
        if (function != null) {
            function.run();
        }
    }

    public static void main(String[] args) {
        CommandSystem commandSystem = new CommandSystem();
        commandSystem.addCommand("function1", () -> {
            // Define the functionality of function1 here
            System.out.println("Executing function1...");
        });
        commandSystem.addCommand("function2", () -> {
            // Define the functionality of function2 here
            System.out.println("Executing function2...");
        });
        // Add more commands as needed...

        Scanner scanner = new Scanner(System.in);
        boolean running = true;

        while (running) {
            System.out.print("Enter a command to execute (or 'exit' to quit): ");
            String input = scanner.nextLine();

            if (input.equalsIgnoreCase("exit")) {
                running = false;
            } else {
                commandSystem.executeCommand(input);
            }
        }

        scanner.close(); // go out of command system
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.logging;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.net.http.HttpResponse.BodyHandlers;
import java.time.Duration;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

import org.json.simple.JSONObject;

import com.fasterxml.jackson.databind.util.JSONPObject;
import com.pathplanner.lib.util.JSONUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Class to poll the /status endpoint of a radio and put the results on NetworkTables.
 * This class assumes NetworkTables is already being auto-logged.
 */
public class RadioLogger {

    // Target radio IP
    private String target;

    // Timing intervals for querying status in seconds
    // This does not include the time taken for the request to arrive
    private double pollRate = 0.25;

    // Timeout length for when the endpoint doesn't respond, in milliseconds
    private long timeout = 2000;


    private HttpClient client = HttpClient.newHttpClient();
    private HttpRequest request;


    public RadioLogger(String ip)
    {
        target = ip;

        request = HttpRequest.newBuilder()
         .uri(URI.create(target))
         .timeout(Duration.ofMillis(timeout))
         .header("Accept","application/json")
         .build();


         // Start querying
         query();

    }


    public void query()
    {
        /*client.sendAsync(request, BodyHandlers.ofString())
         .thenApply(HttpResponse::body)
         .thenAccept((str) -> SmartDashboard.putString("radioLog", str))
         .join();
        */

        client.sendAsync(request, BodyHandlers.ofString())
        
        .thenApply(HttpResponse::body)
        .thenAccept((str) -> SmartDashboard.putString("radioLog", str))
        .exceptionally(this::handleException)
        
        ;
        //.join();
    }

    private Void handleException(Throwable t)
    {
        DriverStation.reportError("Failed to query "+target+": "+t.getLocalizedMessage(), t.getStackTrace());
        SmartDashboard.putString("radioLog", t.getLocalizedMessage());
        return null;// Needs OBJECT Void, not void void
    }

    private void handleLog(String str)
    {
        
    }
    
}
